/*
 *  Quectel BG95 GNSS driver
 *
 *  Copyright (c) 2025  Van Petrosyan
 *  SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "gnss_nmea0183.h"
#include "gnss_nmea0183_match.h"
#include "gnss_parse.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(quectel_bg95_gnss, CONFIG_GNSS_LOG_LEVEL);

enum quectel_bg95_state {
	QUECTEL_BG95_STATE_IDLE,
	QUECTEL_BG95_STATE_AWAIT_MODEM,
	QUECTEL_BG95_STATE_AWAIT_RESUME,
	QUECTEL_BG95_STATE_STARTING,
	QUECTEL_BG95_STATE_ACTIVE,
	QUECTEL_BG95_STATE_STOPPING,
};

enum quectel_bg95_event {
	QUECTEL_BG95_EVENT_RESUME = 0,
	QUECTEL_BG95_EVENT_SUSPEND,
	QUECTEL_BG95_EVENT_TIMEOUT,
	QUECTEL_BG95_EVENT_CHAT_ATTACHED,
	QUECTEL_BG95_EVENT_CHAT_RELEASED,
	QUECTEL_BG95_EVENT_SCRIPT_SUCCESS,
	QUECTEL_BG95_EVENT_SCRIPT_FAILED,
};

struct quectel_bg95_config {
	const struct modem_chat_script *init_chat_script;
	const struct modem_chat_script *shutdown_chat_script;
};

struct quectel_bg95_data {
	struct gnss_nmea0183_match_data match_data;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_QUECTEL_BG95_SATELLITES_COUNT];
#endif
	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[256];
	uint8_t chat_delimiter[2];
	uint8_t *chat_argv[32];

	enum quectel_bg95_state state;
	const struct device *dev;

	struct k_work open_pipe_work;
	struct k_work attach_chat_work;
	struct k_work release_chat_work;
	struct k_work_delayable timeout_work;
	struct modem_pipelink *pipelink;

	/* Power management */
	struct k_sem suspended_sem;

	/* Event dispatcher */
	struct k_work event_dispatch_work;
	uint8_t event_buf[8];
	struct ring_buf event_rb;
	struct k_mutex event_rb_lock;
};

MODEM_CHAT_MATCH_DEFINE(ok_match, "OK", "", NULL);
MODEM_CHAT_MATCHES_DEFINE(abort_matches, MODEM_CHAT_MATCH("ERROR", "", NULL));
MODEM_CHAT_MATCHES_DEFINE(
	bg95_unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("+QGPSGNMEA: $GPGGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("+QGPSGNMEA: $GPRMC,", ",*", gnss_nmea0183_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("+QGPSGNMEA: $GPGSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
);

static void quectel_bg95_enter_state(struct quectel_bg95_data *data, enum quectel_bg95_state state);

static const char *quectel_bg95_state_str(enum quectel_bg95_state state)
{
	switch (state) {
	case QUECTEL_BG95_STATE_IDLE:
		return "idle";
	case QUECTEL_BG95_STATE_AWAIT_MODEM:
		return "await modem";
	case QUECTEL_BG95_STATE_AWAIT_RESUME:
		return "await resume";
	case QUECTEL_BG95_STATE_STARTING:
		return "starting";
	case QUECTEL_BG95_STATE_ACTIVE:
		return "active";
	case QUECTEL_BG95_STATE_STOPPING:
		return "stopping";
	}

	return "";
}

static const char *quectel_bg95_event_str(enum quectel_bg95_event event)
{
	switch (event) {
	case QUECTEL_BG95_EVENT_TIMEOUT:
		return "timeout";
	case QUECTEL_BG95_EVENT_RESUME:
		return "resume";
	case QUECTEL_BG95_EVENT_SUSPEND:
		return "suspend";
	case QUECTEL_BG95_EVENT_CHAT_ATTACHED:
		return "chat attached";
	case QUECTEL_BG95_EVENT_CHAT_RELEASED:
		return "chat released";
	case QUECTEL_BG95_EVENT_SCRIPT_FAILED:
		return "script failed";
	case QUECTEL_BG95_EVENT_SCRIPT_SUCCESS:
		return "script success";
	}

	return "";
}

static void quectel_bg95_log_state_changed(enum quectel_bg95_state last_state,
					   enum quectel_bg95_state new_state)
{
	LOG_DBG("switch from %s to %s", quectel_bg95_state_str(last_state),
		quectel_bg95_state_str(new_state));
}

static void quectel_bg95_log_event(enum quectel_bg95_event evt)
{
	LOG_DBG("event %s", quectel_bg95_event_str(evt));
}

static void quectel_bg95_start_timer(struct quectel_bg95_data *data, k_timeout_t timeout)
{
	k_work_schedule(&data->timeout_work, timeout);
}

static void quectel_bg95_stop_timer(struct quectel_bg95_data *data)
{
	k_work_cancel_delayable(&data->timeout_work);
}

static void quectel_bg95_pipe_callback(struct modem_pipe *pipe, enum modem_pipe_event event,
				       void *user_data)
{
	struct quectel_bg95_data *data = (struct quectel_bg95_data *)(user_data);

	switch (event) {
	case MODEM_PIPE_EVENT_OPENED:
		LOG_DBG("pipe opened");
		k_work_submit(&data->attach_chat_work);
		break;

	default:
		break;
	}
}

static void quectel_bg95_pipelink_callback(struct modem_pipelink *link,
					   enum modem_pipelink_event event, void *user_data)
{
	struct quectel_bg95_data *data = (struct quectel_bg95_data *)(user_data);

	switch (event) {
	case MODEM_PIPELINK_EVENT_CONNECTED:
		LOG_DBG("pipe connected");
		k_work_submit(&data->open_pipe_work);
		break;

	case MODEM_PIPELINK_EVENT_DISCONNECTED:
		LOG_DBG("pipe disconnected");
		k_work_submit(&data->release_chat_work);
		break;

	default:
		break;
	}
}

static int quectel_bg95_on_idle_state_enter(struct quectel_bg95_data *data)
{
	k_sem_give(&data->suspended_sem);
	return 0;
}

static void quectel_bg95_idle_event_handler(struct quectel_bg95_data *data,
					    enum quectel_bg95_event evt)
{
	switch (evt) {
	case QUECTEL_BG95_EVENT_RESUME:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_AWAIT_MODEM);
		break;

	case QUECTEL_BG95_EVENT_CHAT_ATTACHED:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_AWAIT_RESUME);
		break;

	default:
		break;
	}
}

static int quectel_bg95_on_idle_state_leave(struct quectel_bg95_data *data)
{
	return 0;
}

static int quectel_bg95_on_await_resume_state_enter(struct quectel_bg95_data *data)
{
	k_sem_give(&data->suspended_sem);

	return 0;
}

static void quectel_bg95_await_resume_event_handler(struct quectel_bg95_data *data,
						    enum quectel_bg95_event evt)
{
	switch (evt) {
	case QUECTEL_BG95_EVENT_RESUME:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_STARTING);
		break;

	case QUECTEL_BG95_EVENT_CHAT_RELEASED:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int quectel_bg95_on_await_modem_state_leave(struct quectel_bg95_data *data)
{
	return 0;
}

static void quectel_bg95_await_modem_event_handler(struct quectel_bg95_data *data,
						   enum quectel_bg95_event evt)
{
	switch (evt) {
	case QUECTEL_BG95_EVENT_CHAT_ATTACHED:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_STARTING);
		break;

	case QUECTEL_BG95_EVENT_SUSPEND:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int quectel_bg95_on_starting_state_enter(struct quectel_bg95_data *data)
{

	quectel_bg95_start_timer(data, K_NO_WAIT);

	return 0;
}

static void quectel_bg95_starting_event_handler(struct quectel_bg95_data *data,
						enum quectel_bg95_event evt)
{
	const struct quectel_bg95_config *config =
		(const struct quectel_bg95_config *)data->dev->config;

	switch (evt) {
	case QUECTEL_BG95_EVENT_SUSPEND:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_AWAIT_RESUME);
		break;

	case QUECTEL_BG95_EVENT_CHAT_RELEASED:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_AWAIT_MODEM);
		break;

	case QUECTEL_BG95_EVENT_SCRIPT_SUCCESS:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_ACTIVE);
		break;

	case QUECTEL_BG95_EVENT_SCRIPT_FAILED:
		quectel_bg95_start_timer(data, K_SECONDS(5));
		break;

	case QUECTEL_BG95_EVENT_TIMEOUT:

		modem_chat_run_script_async(&data->chat, config->init_chat_script);
		break;

	default:
		break;
	}
}

static void quectel_bg95_active_event_handler(struct quectel_bg95_data *data,
					      enum quectel_bg95_event evt)
{
	switch (evt) {
	case QUECTEL_BG95_EVENT_SUSPEND:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_STOPPING);
		break;

	case QUECTEL_BG95_EVENT_CHAT_RELEASED:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_AWAIT_MODEM);
		break;

	case QUECTEL_BG95_EVENT_TIMEOUT:
		break;

	default:
		break;
	}
}

static int quectel_bg95_on_stopping_state_enter(struct quectel_bg95_data *data)
{
	quectel_bg95_start_timer(data, K_NO_WAIT);

	return 0;
}

static void quectel_bg95_stopping_event_handler(struct quectel_bg95_data *data,
						enum quectel_bg95_event evt)
{
	const struct quectel_bg95_config *config =
		(const struct quectel_bg95_config *)data->dev->config;

	switch (evt) {
	case QUECTEL_BG95_EVENT_SCRIPT_SUCCESS:
	case QUECTEL_BG95_EVENT_SCRIPT_FAILED:
		quectel_bg95_enter_state(data, QUECTEL_BG95_STATE_AWAIT_RESUME);
		break;

	case QUECTEL_BG95_EVENT_TIMEOUT:
		modem_chat_run_script_async(&data->chat, config->shutdown_chat_script);
		break;

	default:
		break;
	}
}

static int quectel_bg95_on_state_enter(struct quectel_bg95_data *data)
{
	int ret;

	switch (data->state) {
	case QUECTEL_BG95_STATE_IDLE:
		ret = quectel_bg95_on_idle_state_enter(data);
		break;

	case QUECTEL_BG95_STATE_AWAIT_RESUME:
		ret = quectel_bg95_on_await_resume_state_enter(data);
		break;

	case QUECTEL_BG95_STATE_STARTING:
		ret = quectel_bg95_on_starting_state_enter(data);
		break;

	case QUECTEL_BG95_STATE_STOPPING:
		ret = quectel_bg95_on_stopping_state_enter(data);
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int quectel_bg95_on_state_leave(struct quectel_bg95_data *data)
{
	int ret;

	switch (data->state) {
	case QUECTEL_BG95_STATE_IDLE:
		ret = quectel_bg95_on_idle_state_leave(data);
		break;

	case QUECTEL_BG95_STATE_AWAIT_MODEM:
		ret = quectel_bg95_on_await_modem_state_leave(data);
		break;

	default:
		ret = 0;
		break;
	}

	return ret;
}

static void quectel_bg95_enter_state(struct quectel_bg95_data *data, enum quectel_bg95_state state)
{
	int ret;

	ret = quectel_bg95_on_state_leave(data);

	if (ret < 0) {
		LOG_WRN("failed to leave state, error: %i", ret);

		return;
	}

	data->state = state;
	ret = quectel_bg95_on_state_enter(data);

	if (ret < 0) {
		LOG_WRN("failed to enter state: {%s}  error: %i", quectel_bg95_state_str(state),
			ret);
	}
}

static void quectel_bg95_event_handler(struct quectel_bg95_data *data, enum quectel_bg95_event evt)
{
	enum quectel_bg95_state state;

	state = data->state;

	quectel_bg95_log_event(evt);

	switch (data->state) {
	case QUECTEL_BG95_STATE_IDLE:
		quectel_bg95_idle_event_handler(data, evt);
		break;
	case QUECTEL_BG95_STATE_AWAIT_MODEM:
		quectel_bg95_await_modem_event_handler(data, evt);
		break;

	case QUECTEL_BG95_STATE_AWAIT_RESUME:
		quectel_bg95_await_resume_event_handler(data, evt);
		break;

	case QUECTEL_BG95_STATE_STARTING:
		quectel_bg95_starting_event_handler(data, evt);
		break;

	case QUECTEL_BG95_STATE_ACTIVE:
		quectel_bg95_active_event_handler(data, evt);
		break;

	case QUECTEL_BG95_STATE_STOPPING:
		quectel_bg95_stopping_event_handler(data, evt);
		break;

	default:
		break;
	}

	if (state != data->state) {
		quectel_bg95_log_state_changed(state, data->state);
	}
}

static void quectel_bg95_event_dispatch_handler(struct k_work *item)
{
	struct quectel_bg95_data *data =
		CONTAINER_OF(item, struct quectel_bg95_data, event_dispatch_work);

	uint8_t events[sizeof(data->event_buf)];
	uint8_t events_cnt;

	k_mutex_lock(&data->event_rb_lock, K_FOREVER);

	events_cnt = (uint8_t)ring_buf_get(&data->event_rb, events, sizeof(data->event_buf));

	k_mutex_unlock(&data->event_rb_lock);

	for (uint8_t i = 0; i < events_cnt; i++) {
		quectel_bg95_event_handler(data, (enum quectel_bg95_event)events[i]);
	}
}

static void quectel_bg95_delegate_event(struct quectel_bg95_data *data, enum quectel_bg95_event evt)
{
	k_mutex_lock(&data->event_rb_lock, K_FOREVER);
	ring_buf_put(&data->event_rb, (uint8_t *)&evt, 1);
	k_mutex_unlock(&data->event_rb_lock);
	k_work_submit(&data->event_dispatch_work);
}

static void quectel_bg95_timeout_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct quectel_bg95_data *data =
		CONTAINER_OF(dwork, struct quectel_bg95_data, timeout_work);

	quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_TIMEOUT);
}

static void quectel_bg95_chat_callback_handler(struct modem_chat *chat,
					       enum modem_chat_script_result result,
					       void *user_data)
{
	struct quectel_bg95_data *data = (struct quectel_bg95_data *)user_data;

	if (result == MODEM_CHAT_SCRIPT_RESULT_SUCCESS) {
		quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_SCRIPT_SUCCESS);
	} else {
		quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_SCRIPT_FAILED);
	}
}

static void quectel_bg95_open_pipe_handler(struct k_work *work)
{
	struct quectel_bg95_data *data =
		CONTAINER_OF(work, struct quectel_bg95_data, open_pipe_work);
	LOG_DBG("opening pipe");

	modem_pipe_attach(modem_pipelink_get_pipe(data->pipelink), quectel_bg95_pipe_callback,
			  data);
	modem_pipe_open_async(modem_pipelink_get_pipe(data->pipelink));
}

static void quectel_bg95_attach_chat_handler(struct k_work *work)
{
	struct quectel_bg95_data *data =
		CONTAINER_OF(work, struct quectel_bg95_data, attach_chat_work);

	modem_chat_attach(&data->chat, modem_pipelink_get_pipe(data->pipelink));
	quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_CHAT_ATTACHED);
	LOG_DBG("chat attached");
}

static void quectel_bg95_release_chat_handler(struct k_work *work)
{
	struct quectel_bg95_data *data =
		CONTAINER_OF(work, struct quectel_bg95_data, release_chat_work);

	modem_chat_release(&data->chat);
	quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_CHAT_RELEASED);

	LOG_DBG("chat released");
}

static DEVICE_API(gnss, gnss_api) = {

};

static int quectel_bg95_init_nmea0183_match(const struct device *dev)
{
	struct quectel_bg95_data *data = dev->data;

	const struct gnss_nmea0183_match_config match_config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};

	return gnss_nmea0183_match_init(&data->match_data, &match_config);
}

static int gnss_quectel_bg95_init_chat(const struct device *dev)
{
	struct quectel_bg95_data *data = dev->data;

	const struct modem_chat_config chat_config = {
		.user_data = data,
		.receive_buf = data->chat_receive_buf,
		.receive_buf_size = sizeof(data->chat_receive_buf),
		.delimiter = data->chat_delimiter,
		.delimiter_size = ARRAY_SIZE(data->chat_delimiter),
		.filter = NULL,
		.filter_size = 0,
		.argv = data->chat_argv,
		.argv_size = ARRAY_SIZE(data->chat_argv),
		.unsol_matches = bg95_unsol_matches,
		.unsol_matches_size = ARRAY_SIZE(bg95_unsol_matches),
	};

	return modem_chat_init(&data->chat, &chat_config);
}

static int quectel_bg95_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct quectel_bg95_data *data = (struct quectel_bg95_data *)dev->data;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_RESUME);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		quectel_bg95_delegate_event(data, QUECTEL_BG95_EVENT_SUSPEND);
		ret = k_sem_take(&data->suspended_sem, K_SECONDS(5));
		break;

	default:
		break;
	}

	return ret;
}

static int quectel_bg95_init(const struct device *dev)
{
	struct quectel_bg95_data *data = dev->data;
	int ret;

	data->dev = dev;
	data->state = QUECTEL_BG95_STATE_IDLE;

	k_sem_init(&data->suspended_sem, 0, 1);
	k_mutex_init(&data->event_rb_lock);
	k_work_init_delayable(&data->timeout_work, quectel_bg95_timeout_handler);

	k_work_init(&data->event_dispatch_work, quectel_bg95_event_dispatch_handler);
	ring_buf_init(&data->event_rb, sizeof(data->event_buf), data->event_buf);

	k_work_init(&data->open_pipe_work, quectel_bg95_open_pipe_handler);
	k_work_init(&data->attach_chat_work, quectel_bg95_attach_chat_handler);
	k_work_init(&data->release_chat_work, quectel_bg95_release_chat_handler);

	modem_pipelink_attach(data->pipelink, quectel_bg95_pipelink_callback, data);

	ret = quectel_bg95_init_nmea0183_match(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize NMEA0183 match");
		return ret;
	}

	ret = gnss_quectel_bg95_init_chat(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize modem chat");
		return ret;
	}
#ifndef CONFIG_PM_DEVICE
	quectel_bg95_delegate_event(data, MODEM_CELLULAR_EVENT_RESUME);
#else
	pm_device_init_suspended(dev);
#endif /* CONFIG_PM_DEVICE */

	return 0;
}

MODEM_CHAT_SCRIPT_CMDS_DEFINE(gnss_quectel_bg95_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPSXTRA=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPSCFG=\"priority\",0", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPSCFG=\"nmeafmt\",1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPSCFG=\"nmeasrc\",1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPS=1", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(gnss_quectel_bg95_init_chat_script,
			 gnss_quectel_bg95_init_chat_script_cmds, abort_matches,
			 quectel_bg95_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(gnss_quectel_bg95_shutdown_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPSCFG=\"priority\",1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QGPSEND", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(gnss_quectel_bg95_shutdown_chat_script,
			 gnss_quectel_bg95_shutdown_chat_script_cmds, abort_matches,
			 quectel_bg95_chat_callback_handler, 4);

#define _USER_PIPE(idx) user_pipe_##idx
#define USER_PIPE(idx)  _USER_PIPE(idx)
#define PIPE_NODE(inst) DT_PHANDLE(DT_DRV_INST(inst), modem_pipe)
#define PIPE_IDX(inst)  DT_PROP(PIPE_NODE(inst), index)
#define PIPE_NAME(inst) USER_PIPE(PIPE_IDX(inst))

#define BG95_INST_NAME(inst, name) _CONCAT(_CONCAT(_CONCAT(name, _), DT_DRV_COMPAT), inst)

#define BG95_DEVICE(inst)                                                                          \
	/* Forward‑declare the pipelink symbol created by the modem driver */                      \
	MODEM_PIPELINK_DT_DECLARE(DT_PARENT(PIPE_NODE(inst)), PIPE_NAME(inst));                    \
                                                                                                   \
	static const struct quectel_bg95_config BG95_INST_NAME(inst, config) = {                   \
		.init_chat_script = &gnss_quectel_bg95_init_chat_script,                           \
		.shutdown_chat_script = &gnss_quectel_bg95_shutdown_chat_script,                   \
	};                                                                                         \
                                                                                                   \
	static struct quectel_bg95_data BG95_INST_NAME(inst, data) = {                             \
		.chat_delimiter = {'\r', '\n'},                                                    \
		.pipelink = MODEM_PIPELINK_DT_GET(DT_PARENT(PIPE_NODE(inst)), PIPE_NAME(inst)),    \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, quectel_bg95_pm_action);                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, quectel_bg95_init, PM_DEVICE_DT_INST_GET(inst),                \
			      &BG95_INST_NAME(inst, data), &BG95_INST_NAME(inst, config),          \
			      POST_KERNEL, CONFIG_GNSS_QUECTEL_BG95_INIT_PRIORITY, &gnss_api);

#define DT_DRV_COMPAT quectel_bg95_gnss
DT_INST_FOREACH_STATUS_OKAY(BG95_DEVICE)
#undef DT_DRV_COMPAT
