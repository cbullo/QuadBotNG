/*
 * lws-minimal-http-server-sse
 *
 * Written in 2010-2019 by Andy Green <andy@warmcat.com>
 *
 * This file is made available under the Creative Commons CC0 1.0
 * Universal Public Domain Dedication.
 *
 * This demonstrates a minimal http server that can serve both normal static
 * content and server-side event connections.
 *
 * To keep it simple, it serves the static stuff from the subdirectory
 * "./mount-origin" of the directory it was started in.
 *
 * You can change that by changing mount.origin below.
 */

#include "server.h"

#include <memory>
#include <regex>
#include <string>

#include "src/odroid_controller/controller.h"

/*
 * Unlike ws, http is a stateless protocol.  This pss only exists for the
 * duration of a single http transaction.  With http/1.1 keep-alive and http/2,
 * that is unrelated to (shorter than) the lifetime of the network connection.
 */

enum class ValueType {
  Theta,
  Gamma,
  Z,
  AngleI,
  AngleO,
  AngleZ,
  TemperatureI,
  TemperatureO,
  TemperatureZ
};

struct pss {
  Leg *leg = nullptr;
  ValueType type;
};

struct LogLegData {
  const char *theta;
  const char *gamma;
  const char *z;
  const char *temperature_i;
  const char *temperature_o;
  const char *temperature_z;
};

static const lws_struct_map_t LSMLogLegData[] = {
    LSM_STRING_PTR(LogLegData, theta, "theta"),
    LSM_STRING_PTR(LogLegData, gamma, "gamma"),
    LSM_STRING_PTR(LogLegData, z, "z"),
    LSM_STRING_PTR(LogLegData, temperature_i, "temperature_i"),
    LSM_STRING_PTR(LogLegData, temperature_o, "temperature_o"),
    LSM_STRING_PTR(LogLegData, temperature_z, "temperature_z")};

struct LogEventData {
  LogLegData *bl = nullptr;
  LogLegData *br = nullptr;
  LogLegData *fl = nullptr;
  LogLegData *fr = nullptr;
  char *timestamp;
};

static const lws_struct_map_t LSMLogEventData[] = {
    LSM_CHILD_PTR(LogEventData, bl, LogLegData, NULL, LSMLogLegData, "bl"),
    LSM_CHILD_PTR(LogEventData, br, LogLegData, NULL, LSMLogLegData, "br"),
    LSM_CHILD_PTR(LogEventData, fl, LogLegData, NULL, LSMLogLegData, "fl"),
    LSM_CHILD_PTR(LogEventData, fr, LogLegData, NULL, LSMLogLegData, "fr"),
    LSM_STRING_PTR(LogEventData, timestamp, "timestamp")};

static const lws_struct_map_t Schema[] = {
    LSM_SCHEMA(LSMLogEventData, NULL, LSMLogEventData, "data")};

#define USECS_REPORT (200 * LWS_USEC_PER_SEC / 1000)

static int callback_sse(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len) {
  struct pss *pss = (struct pss *)user;
  uint8_t buf[LWS_PRE + LWS_RECOMMENDED_MIN_HEADER_SPACE],
      *start = &buf[LWS_PRE], *p = start, *end = &buf[sizeof(buf) - 1];

  lws_context *lws_cx = lws_get_context(wsi);
  auto controller = (Controller *)(lws_context_user(lws_cx));

  switch (reason) {
    case LWS_CALLBACK_HTTP: {
      /*
       * `in` contains the url part after our mountpoint /sse, if any
       * you can use this to determine what data to return and store
       * that in the pss
       */
      // lwsl_notice("%s: LWS_CALLBACK_HTTP: '%s'\n", __func__, (const char
      // *)in);

      // std::string path = std::string(static_cast<const char *>(in));

      // std::regex subpath_regex("(/([^/]+))");

      // int index = 0;

      // std::string leg;
      // std::string value;

      // std::smatch base_match;

      // if (std::regex_match(path, base_match, subpath_regex)) {
      //   // The first sub_match is the whole string; the next
      //   // sub_match is the first parenthesized expression.
      //   std::cout << base_match.size() << std::endl;
      //   if (base_match.size() == 4) {
      //     std::ssub_match value_sub_match = base_match[1];
      //     value = value_sub_match.str();

      //     std::ssub_match leg_sub_match = base_match[3];
      //     leg = leg_sub_match.str();
      //   }
      // }

      // std::cout << path << std::endl;
      // int sstart = 0;
      // int send = 0;
      // if ((sstart = path.find_first_of('/', sstart)) >= 0) {
      //   sstart = sstart + 1;
      //   send = path.find_first_of('/', sstart);

      //   value = path.substr(sstart, send - sstart);
      //   leg = path.substr(send + 1);
      // }

      // std::transform(leg.begin(), leg.end(), leg.begin(),
      //                [](unsigned char c) { return std::tolower(c); });
      // std::transform(value.begin(), value.end(), value.begin(),
      //                [](unsigned char c) { return std::tolower(c); });

      // std::cout << leg << std::endl;
      // std::cout << value << std::endl;

      // if (leg == "bl") {
      //   pss->leg = controller->GetLegs().bl;
      // } else if (leg == "br") {
      //   pss->leg = controller->GetLegs().br;
      // } else if (leg == "fl") {
      //   pss->leg = controller->GetLegs().fl;
      // } else if (leg == "fr") {
      //   pss->leg = controller->GetLegs().fr;
      // }

      // if (leg == "tempi") {
      //   pss->type = ValueType::TemperatureI;
      // } else if (leg == "tempo") {
      //   pss->type = ValueType::TemperatureO;
      // } else if (leg == "tempz") {
      //   pss->type = ValueType::TemperatureZ;
      // } else if (leg == "theta") {
      //   pss->type = ValueType::Theta;
      // } else if (leg == "gamma") {
      //   pss->type = ValueType::Gamma;
      // } else if (leg == "z") {
      //   pss->type = ValueType::Z;
      // } else if (leg == "anglei") {
      //   pss->type = ValueType::AngleI;
      // } else if (leg == "angleo") {
      //   pss->type = ValueType::AngleO;
      // } else if (leg == "anglez") {
      //   pss->type = ValueType::AngleZ;
      // }

      // pss->established = time(NULL);

      /* SSE requires a response with this content-type */

      if (lws_add_http_common_headers(wsi, HTTP_STATUS_OK, "text/event-stream",
                                      LWS_ILLEGAL_HTTP_CONTENT_LEN, &p, end))
        return 1;

      if (lws_finalize_write_http_header(wsi, start, &p, end)) return 1;

      /*
       * This tells lws we are no longer a normal http stream,
       * but are an "immortal" (plus or minus whatever timeout you
       * set on it afterwards) SSE stream.  In http/2 case that also
       * stops idle timeouts being applied to the network connection
       * while this wsi is still open.
       */
      lws_http_mark_sse(wsi);

      /* write the body separately */

      lws_callback_on_writable(wsi);
    }
      return 0;

    case LWS_CALLBACK_HTTP_WRITEABLE: {
      lwsl_notice("%s: LWS_CALLBACK_HTTP_WRITEABLE\n", __func__);

      // if (!pss || !pss->leg) {
      //   std::cout << "no leg" << std::endl;
      //   break;
      // }

      /*
       * to keep this demo as simple as possible, each client has his
       * own private data and timer.
       */

      // float value = 0.f;
      // switch (pss->type) {
      //   case ValueType::TemperatureI:
      //     value = pss->leg->GetMotorI()->GetTemperature();
      //     break;
      //   case ValueType::TemperatureO:
      //     value = pss->leg->GetMotorO()->GetTemperature();
      //     break;
      //   case ValueType::TemperatureZ:
      //     value = pss->leg->GetMotorZ()->GetTemperature();
      //     break;
      //   case ValueType::Gamma:
      //     value = pss->leg->GetGamma();
      //     break;
      //   case ValueType::Theta:
      //     value = pss->leg->GetTheta();
      //     break;
      //   case ValueType::Z:
      //     value = pss->leg->GetAngleZ();
      //     break;
      //   case ValueType::AngleI:
      //     value = pss->leg->GetAngleI();
      //     break;
      //   case ValueType::AngleO:
      //     value = pss->leg->GetAngleO();
      //     break;
      //   case ValueType::AngleZ:
      //     value = pss->leg->GetAngleZ();
      //     break;
      // }

      LogLegData bl;
      LogLegData br;
      LogLegData fl;
      LogLegData fr;
      LogEventData event_data;
      event_data.bl = &bl;
      event_data.br = &br;
      event_data.fl = &fl;
      event_data.fr = &fr;

      std::string gamma_str_bl =
          std::to_string(controller->GetLegs().bl->GetGamma());
      event_data.bl->gamma = const_cast<char *>(gamma_str_bl.c_str());
      std::string theta_str_bl =
          std::to_string(controller->GetLegs().bl->GetTheta());
      event_data.bl->theta = const_cast<char *>(theta_str_bl.c_str());
      std::string z_str_bl =
          std::to_string(controller->GetLegs().bl->GetAngleZ());
      event_data.bl->z = const_cast<char *>(z_str_bl.c_str());
      std::string temperature_i_str_bl = std::to_string(
          controller->GetLegs().bl->GetMotorI()->GetTemperature());
      event_data.bl->temperature_i =
          const_cast<char *>(temperature_i_str_bl.c_str());
      std::string temperature_o_str_bl = std::to_string(
          controller->GetLegs().bl->GetMotorO()->GetTemperature());
      event_data.bl->temperature_o =
          const_cast<char *>(temperature_o_str_bl.c_str());
      std::string temperature_z_str_bl = std::to_string(
          controller->GetLegs().bl->GetMotorZ()->GetTemperature());
      event_data.bl->temperature_z =
          const_cast<char *>(temperature_z_str_bl.c_str());

      std::string gamma_str_br =
          std::to_string(controller->GetLegs().br->GetGamma());
      event_data.br->gamma = const_cast<char *>(gamma_str_br.c_str());
      std::string theta_str_br =
          std::to_string(controller->GetLegs().br->GetTheta());
      event_data.br->theta = const_cast<char *>(theta_str_br.c_str());
      std::string z_str_br =
          std::to_string(controller->GetLegs().br->GetAngleZ());
      event_data.br->z = const_cast<char *>(z_str_br.c_str());
      std::string temperature_i_str_br = std::to_string(
          controller->GetLegs().br->GetMotorI()->GetTemperature());
      event_data.br->temperature_i =
          const_cast<char *>(temperature_i_str_br.c_str());
      std::string temperature_o_str_br = std::to_string(
          controller->GetLegs().br->GetMotorO()->GetTemperature());
      event_data.br->temperature_o =
          const_cast<char *>(temperature_o_str_br.c_str());
      std::string temperature_z_str_br = std::to_string(
          controller->GetLegs().br->GetMotorZ()->GetTemperature());
      event_data.br->temperature_z =
          const_cast<char *>(temperature_z_str_br.c_str());

      std::string gamma_str_fl =
          std::to_string(controller->GetLegs().fl->GetGamma());
      event_data.fl->gamma = const_cast<char *>(gamma_str_fl.c_str());
      std::string theta_str_fl =
          std::to_string(controller->GetLegs().fl->GetTheta());
      event_data.fl->theta = const_cast<char *>(theta_str_fl.c_str());
      std::string z_str_fl =
          std::to_string(controller->GetLegs().fl->GetAngleZ());
      event_data.fl->z = const_cast<char *>(z_str_fl.c_str());
      std::string temperature_i_str_fl = std::to_string(
          controller->GetLegs().fl->GetMotorI()->GetTemperature());
      event_data.fl->temperature_i =
          const_cast<char *>(temperature_i_str_fl.c_str());
      std::string temperature_o_str_fl = std::to_string(
          controller->GetLegs().fl->GetMotorO()->GetTemperature());
      event_data.fl->temperature_o =
          const_cast<char *>(temperature_o_str_fl.c_str());
      std::string temperature_z_str_fl = std::to_string(
          controller->GetLegs().fl->GetMotorZ()->GetTemperature());
      event_data.fl->temperature_z =
          const_cast<char *>(temperature_z_str_fl.c_str());

      std::string gamma_str_fr =
          std::to_string(controller->GetLegs().fr->GetGamma());
      event_data.fr->gamma = const_cast<char *>(gamma_str_fr.c_str());
      std::string theta_str_fr =
          std::to_string(controller->GetLegs().fr->GetTheta());
      event_data.fr->theta = const_cast<char *>(theta_str_fr.c_str());
      std::string z_str_fr =
          std::to_string(controller->GetLegs().fr->GetAngleZ());
      event_data.fr->z = const_cast<char *>(z_str_fr.c_str());
      std::string temperature_i_str_fr = std::to_string(
          controller->GetLegs().fr->GetMotorI()->GetTemperature());
      event_data.fr->temperature_i =
          const_cast<char *>(temperature_i_str_fr.c_str());
      std::string temperature_o_str_fr = std::to_string(
          controller->GetLegs().fr->GetMotorO()->GetTemperature());
      event_data.fr->temperature_o =
          const_cast<char *>(temperature_o_str_fr.c_str());
      std::string temperature_z_str_fr = std::to_string(
          controller->GetLegs().fr->GetMotorZ()->GetTemperature());
      event_data.fr->temperature_z =
          const_cast<char *>(temperature_z_str_fr.c_str());

      auto serializer = lws_struct_json_serialize_create(
          Schema, LWS_ARRAY_SIZE(Schema), LSSERJ_FLAG_OMIT_SCHEMA, &event_data);

      float duration = controller->GetDuration();

      std::string duration_str = std::to_string(duration);
      event_data.timestamp = const_cast<char *>(duration_str.c_str());

      if (!serializer) {
        std::cout << "Failed to serialize" << std::endl;
        return -1;
      }

      p += lws_snprintf((char *)p, lws_ptr_diff_size_t(end, p), "data: ");

      size_t written = 0;
      lws_struct_json_serialize(serializer, (uint8_t *)p,
                                lws_ptr_diff_size_t(end, p), &written);
      p += written;

      lws_struct_json_serialize_destroy(&serializer);

      p += lws_snprintf((char *)p, lws_ptr_diff_size_t(end, p),
                        "\x0d\x0a\x0d\x0a");

      // std::cout.write((const char *)start, p - start);
      // std::cout << std::endl;
      // // p += lws_snprintf((char *)p, lws_ptr_diff_size_t(end, p),
      // "timestamp:
      // // %f\x0d\x0a\x0d\x0a", duration);

      if (lws_write(wsi, (uint8_t *)start, lws_ptr_diff_size_t(p, start),
                    LWS_WRITE_HTTP) != lws_ptr_diff(p, start)) {
        return 1;
      }
      lws_set_timer_usecs(wsi, USECS_REPORT);
    }
      return 0;

    case LWS_CALLBACK_TIMER:

      lwsl_notice("%s: LWS_CALLBACK_TIMER\n", __func__);
      lws_callback_on_writable(wsi);

      return 0;

    default:
      break;
  }

  return lws_callback_http_dummy(wsi, reason, user, in, len);
}

int WebServer::Init(const Controller *controller) {
  struct lws_context_creation_info info;

  const char *p;
  int n = 0, logs = LLL_USER | LLL_ERR | LLL_WARN | LLL_NOTICE
      /* for LLL_ verbosity above NOTICE to be built into lws,
       * lws must have been configured and built with
       * -DCMAKE_BUILD_TYPE=DEBUG instead of =RELEASE */
      /* | LLL_INFO */ /* | LLL_PARSER */ /* | LLL_HEADER */
      /* | LLL_EXT */ /* | LLL_CLIENT */  /* | LLL_LATENCY */
      /* | LLL_DEBUG */;

  static struct lws_protocols protocols[] = {
      {"http", lws_callback_http_dummy, 0, 0, 0, NULL, 0},
      {"sse", callback_sse, sizeof(struct pss), 0, 0, NULL, 0},
      LWS_PROTOCOL_LIST_TERM};

  /* override the default mount for /sse in the URL space */

  static const struct lws_http_mount mount_sse = {
      /* .mount_next */ NULL,   /* linked-list "next" */
      /* .mountpoint */ "/sse", /* mountpoint URL */
      /* .origin */ NULL,       /* protocol */
      /* .def */ NULL,
      /* .protocol */ "sse",
      /* .cgienv */ NULL,
      /* .extra_mimetypes */ NULL,
      /* .interpret */ NULL,
      /* .cgi_timeout */ 0,
      /* .cache_max_age */ 0,
      /* .auth_mask */ 0,
      /* .cache_reusable */ 0,
      /* .cache_revalidate */ 0,
      /* .cache_intermediaries */ 0,
      /* .origin_protocol */ LWSMPRO_CALLBACK, /* dynamic */
      /* .mountpoint_len */ 4,                 /* char count */
      /* .basic_auth_login_file */ NULL,
  };

  /* default mount serves the URL space from ./mount-origin */

  static const struct lws_http_mount mount = {
      /* .mount_next */ &mount_sse, /* linked-list "next" */
      /* .mountpoint */ "/",        /* mountpoint URL */
      /* .origin */ "./public",     /* serve from dir */
      /* .def */ "index.html",      /* default filename */
      /* .protocol */ NULL,
      /* .cgienv */ NULL,
      /* .extra_mimetypes */ NULL,
      /* .interpret */ NULL,
      /* .cgi_timeout */ 0,
      /* .cache_max_age */ 0,
      /* .auth_mask */ 0,
      /* .cache_reusable */ 0,
      /* .cache_revalidate */ 0,
      /* .cache_intermediaries */ 0,
      /* .origin_protocol */ LWSMPRO_FILE, /* files in a dir */
      /* .mountpoint_len */ 1,             /* char count */
      /* .basic_auth_login_file */ NULL,
  };

  lws_set_log_level(logs, NULL);
  lwsl_user(
      "LWS minimal http Server-Side Events | visit http://localhost:7681\n");

  memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */

  info.protocols = protocols;
  info.mounts = &mount;
  info.options = 0;
  info.port = 8080;
  info.user = const_cast<Controller *>(controller);

  context_ = lws_create_context(&info);
  if (!context_) {
    lwsl_err("lws init failed\n");
    return 1;
  }

  controller_ = controller;

  communication_thread_ = std::make_unique<std::thread>(&WebServer::Tick, this);
}

void WebServer::Tick() {
  int n = 0;
  while (n >= 0) {
    n = lws_service(context_, 0);
  }

  lws_context_destroy(context_);
}
