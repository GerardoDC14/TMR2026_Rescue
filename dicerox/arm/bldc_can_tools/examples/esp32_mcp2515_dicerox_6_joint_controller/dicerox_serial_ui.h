#pragma once
#include "dicerox_joint_control.h"

using namespace odrive_can_sniffer;

// ── Help ──────────────────────────────────────────────────────────────────────

inline void printHelp() {
  // Motion commands
  Serial.println("Commands:");
  Serial.println("  help | h");
  Serial.println("     show this help");
  Serial.println("  list");
  Serial.println("     list all six joints");
  Serial.println("  use TARGET");
  Serial.println("     set active target, for example: use joint1, use joint4, use ze300");
  Serial.println("  status [TARGET|all]");
  Serial.println("     print cached status");
  Serial.println("  pos [TARGET|all]");
  Serial.println("     read current position");
  Serial.println("  zero [TARGET|all]");
  Serial.println("     capture current position as software zero");
  Serial.println("  on [TARGET|all]");
  Serial.println("     ODrive: auto bringup + zero, Mixed: enable + zero");
  Serial.println("  off [TARGET|all]");
  Serial.println("     ODrive: IDLE, Mixed: disable output");
  Serial.println("  stop [TARGET|all]");
  Serial.println("     ODrive: maps to IDLE, Mixed: stop/disable");
  Serial.println("  goto DEG");
  Serial.println("  goto TARGET DEG");
  Serial.println("     command output-side angle relative to software zero");
  Serial.println("  speed DPS");
  Serial.println("  speed TARGET DPS");
  Serial.println("     set output-side speed for joints 4-6");
  // Debug commands
  Serial.println("  raw on|off");
  Serial.println("     toggle raw CAN RX printing");
  Serial.println("  telemetry on|off");
  Serial.println("     toggle ODrive encoder telemetry printing");
  Serial.println("  stream on|off");
  Serial.println("     toggle ODrive 20 Hz target resend");
  Serial.println("  mode normal|listen|loopback");
  Serial.println("     set MCP2515 mode");
  Serial.println("  osc 8|16");
  Serial.println("     set MCP2515 oscillator assumption");
  Serial.println("  selftest");
  Serial.println("     send 123#DEADBEEF");
  Serial.println("Single-token shortcuts:");
  Serial.println("  1..6   select joint1..joint6");
  Serial.println("  p      print config");
  Serial.println("  b      run `on` on the active target");
  Serial.println("  z      run `zero` on the active target");
  Serial.println("  o      run `status` on the active target");
  Serial.println("  a      toggle raw RX debug");
  Serial.println("  v      toggle ODrive telemetry");
  Serial.println("  k      toggle ODrive target streaming");
  Serial.println("  s      send selftest frame");
  Serial.println("  c/x/e/r only when active target is an ODrive joint");
  Serial.println("  g <deg> active target goto");
  Serial.println("  t <turns> active ODrive target motor-turn debug");
}

// ── Command handlers ──────────────────────────────────────────────────────────

inline void handleUseCommand(int argc, char *argv[]) {
  if (argc != 2) {
    Serial.println("usage: use TARGET");
    return;
  }
  size_t target_index = 0;
  if (!findUnifiedTargetByToken(argv[1], &target_index)) {
    printUnknownTarget(argv[1]);
    return;
  }
  selectActiveTarget(target_index);
}

inline void handleStatusCommand(int argc, char *argv[]) {
  if (argc == 1 || (argc == 2 && equalsIgnoreCase(argv[1], "all"))) {
    printAllStatus();
    return;
  }
  printSingleTargetStatus(argv[1]);
}

inline void handleSimpleTargetAction(int argc,
                                     char *argv[],
                                     const char *command_name,
                                     bool (*fn)(size_t)) {
  if (argc == 1) {
    fn(g_active_target_index);
    return;
  }
  if (argc != 2) {
    Serial.printf("usage: %s [TARGET|all]\n", command_name);
    return;
  }
  if (equalsIgnoreCase(argv[1], "all")) {
    runActionOnAll(fn);
    return;
  }
  size_t target_index = 0;
  if (!findUnifiedTargetByToken(argv[1], &target_index)) {
    printUnknownTarget(argv[1]);
    return;
  }
  fn(target_index);
}

inline void handleSpeedCommand(int argc, char *argv[]) {
  size_t      target_index = g_active_target_index;
  const char *value_token  = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findUnifiedTargetByToken(argv[1], &target_index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: speed DPS  or  speed TARGET DPS");
    return;
  }

  double output_dps = 0.0;
  if (!parseDoubleToken(value_token, &output_dps)) {
    Serial.println("invalid speed value, expected something like: speed joint6 180");
    return;
  }
  speedUnifiedMotor(target_index, output_dps);
}

inline void handleGotoCommand(int argc, char *argv[]) {
  size_t      target_index = g_active_target_index;
  const char *value_token  = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findUnifiedTargetByToken(argv[1], &target_index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: goto DEG  or  goto TARGET DEG");
    return;
  }

  double output_deg = 0.0;
  if (!parseDoubleToken(value_token, &output_deg)) {
    Serial.println("invalid goto angle, expected something like: goto joint1 30 or goto -15");
    return;
  }
  gotoUnifiedMotor(target_index, output_deg);
}

inline void handleRawCommand(int argc, char *argv[]) {
  if (argc == 1) {
    g_show_all_frames = !g_show_all_frames;
  } else if (argc == 2 && (equalsIgnoreCase(argv[1], "on") || equalsIgnoreCase(argv[1], "off"))) {
    g_show_all_frames = equalsIgnoreCase(argv[1], "on");
  } else {
    Serial.println("usage: raw [on|off]");
    return;
  }
  Serial.printf("raw_rx_debug = %s\n", g_show_all_frames ? "ON" : "OFF");
}

inline void handleTelemetryCommand(int argc, char *argv[]) {
  if (argc == 1) {
    g_print_odrive_telemetry = !g_print_odrive_telemetry;
  } else if (argc == 2 && (equalsIgnoreCase(argv[1], "on") || equalsIgnoreCase(argv[1], "off"))) {
    g_print_odrive_telemetry = equalsIgnoreCase(argv[1], "on");
  } else {
    Serial.println("usage: telemetry [on|off]");
    return;
  }
  for (size_t index = 0; index < SUPPORTED_NODE_COUNT; ++index) {
    g_odrive_states[index].lastEncoderTelemetryPrintMs = 0;
  }
  Serial.printf("odrive_telemetry = %s\n", g_print_odrive_telemetry ? "ON" : "OFF");
}

inline void handleStreamCommand(int argc, char *argv[]) {
  if (argc == 1) {
    g_stream_odrive_targets = !g_stream_odrive_targets;
  } else if (argc == 2 && (equalsIgnoreCase(argv[1], "on") || equalsIgnoreCase(argv[1], "off"))) {
    g_stream_odrive_targets = equalsIgnoreCase(argv[1], "on");
  } else {
    Serial.println("usage: stream [on|off]");
    return;
  }
  Serial.printf("odrive_target_stream = %s\n", g_stream_odrive_targets ? "ON" : "OFF");
}

inline void handleModeCommand(int argc, char *argv[]) {
  if (argc != 2) {
    Serial.println("usage: mode normal|listen|loopback");
    return;
  }

  if (equalsIgnoreCase(argv[1], "normal") || equalsIgnoreCase(argv[1], "n")) {
    g_mode = CanMode::Normal;
  } else if (equalsIgnoreCase(argv[1], "listen") || equalsIgnoreCase(argv[1], "listen-only") ||
             equalsIgnoreCase(argv[1], "i")) {
    g_mode = CanMode::ListenOnly;
  } else if (equalsIgnoreCase(argv[1], "loopback") || equalsIgnoreCase(argv[1], "l")) {
    g_mode = CanMode::Loopback;
  } else {
    Serial.println("usage: mode normal|listen|loopback");
    return;
  }

  configureCan();
  printConfig();
}

inline void handleOscCommand(int argc, char *argv[]) {
  if (argc != 2) {
    Serial.println("usage: osc 8|16");
    return;
  }

  if (equalsIgnoreCase(argv[1], "8")) {
    g_oscillator = Oscillator::MHz8;
  } else if (equalsIgnoreCase(argv[1], "16") || equalsIgnoreCase(argv[1], "6")) {
    g_oscillator = Oscillator::MHz16;
  } else {
    Serial.println("usage: osc 8|16");
    return;
  }

  configureCan();
  printConfig();
}

// ── Single-token shortcut dispatcher ─────────────────────────────────────────

inline void handleSingleTokenShortcut(const char *token) {
  if (token == nullptr || token[0] == '\0') {
    return;
  }

  if (equalsIgnoreCase(token, "1")) { selectActiveTarget(0); return; }
  if (equalsIgnoreCase(token, "2")) { selectActiveTarget(1); return; }
  if (equalsIgnoreCase(token, "3")) { selectActiveTarget(2); return; }
  if (equalsIgnoreCase(token, "4")) { selectActiveTarget(3); return; }
  if (equalsIgnoreCase(token, "5")) { selectActiveTarget(4); return; }
  if (equalsIgnoreCase(token, "6")) { selectActiveTarget(5); return; }

  if (equalsIgnoreCase(token, "p")) {
    printConfig();
    return;
  }
  if (equalsIgnoreCase(token, "b")) {
    onUnifiedMotor(g_active_target_index);
    return;
  }
  if (equalsIgnoreCase(token, "z")) {
    zeroUnifiedMotor(g_active_target_index);
    return;
  }
  if (equalsIgnoreCase(token, "o")) {
    printTargetStatusLine(g_active_target_index);
    return;
  }
  if (equalsIgnoreCase(token, "a")) {
    g_show_all_frames = !g_show_all_frames;
    Serial.printf("raw_rx_debug = %s\n", g_show_all_frames ? "ON" : "OFF");
    return;
  }
  if (equalsIgnoreCase(token, "v")) {
    g_print_odrive_telemetry = !g_print_odrive_telemetry;
    for (size_t index = 0; index < SUPPORTED_NODE_COUNT; ++index) {
      g_odrive_states[index].lastEncoderTelemetryPrintMs = 0;
    }
    Serial.printf("odrive_telemetry = %s\n", g_print_odrive_telemetry ? "ON" : "OFF");
    return;
  }
  if (equalsIgnoreCase(token, "k")) {
    g_stream_odrive_targets = !g_stream_odrive_targets;
    Serial.printf("odrive_target_stream = %s\n", g_stream_odrive_targets ? "ON" : "OFF");
    return;
  }
  if (equalsIgnoreCase(token, "s")) {
    sendSelfTestFrame();
    return;
  }
  if (equalsIgnoreCase(token, "n")) {
    g_mode = CanMode::Normal;
    configureCan();
    printConfig();
    return;
  }
  if (equalsIgnoreCase(token, "i")) {
    g_mode = CanMode::ListenOnly;
    configureCan();
    printConfig();
    return;
  }
  if (equalsIgnoreCase(token, "l")) {
    g_mode = CanMode::Loopback;
    configureCan();
    printConfig();
    return;
  }
  if (equalsIgnoreCase(token, "8")) {
    g_oscillator = Oscillator::MHz8;
    configureCan();
    printConfig();
    return;
  }
  if (equalsIgnoreCase(token, "c")) {
    if (!activeTargetIsOdrive()) {
      Serial.println("c is ODrive-only; select joint1, joint2, or joint3 first.");
      return;
    }
    requestOdriveClosedLoopAndConfirm(activeTarget().odrive_node_id, CLOSED_LOOP_CONFIRM_TIMEOUT_MS);
    return;
  }
  if (equalsIgnoreCase(token, "x")) {
    if (!activeTargetIsOdrive()) {
      Serial.println("x is ODrive-only; select joint1, joint2, or joint3 first.");
      return;
    }
    setOdriveIdle(activeTarget().odrive_node_id);
    return;
  }
  if (equalsIgnoreCase(token, "e")) {
    if (!activeTargetIsOdrive()) {
      Serial.println("e is ODrive-only; select joint1, joint2, or joint3 first.");
      return;
    }
    if (!ensureCanNormalMode("odrive encoder request")) {
      return;
    }
    sendOdriveRemoteRequest(activeTarget().odrive_node_id, CMD_GET_ENCODER_ESTIMATES, 8, "Requested Get_Encoder_Estimates", true);
    return;
  }
  if (equalsIgnoreCase(token, "r")) {
    if (!activeTargetIsOdrive()) {
      Serial.println("r is ODrive-only; select joint1, joint2, or joint3 first.");
      return;
    }
    if (!ensureCanNormalMode("odrive error request")) {
      return;
    }
    sendOdriveRemoteRequest(activeTarget().odrive_node_id, CMD_GET_ERROR, 8, "Requested Get_Error", true);
    return;
  }
  if (equalsIgnoreCase(token, "help") || equalsIgnoreCase(token, "h") || equalsIgnoreCase(token, "?")) {
    printHelp();
    return;
  }

  Serial.printf("unknown command: %s\n", token);
  printHelp();
}

// ── Line dispatcher ───────────────────────────────────────────────────────────

inline void handleSerialLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (*trimmed == '\0') {
    return;
  }

  char *argv[5] = {};
  const int argc = tokenize(trimmed, argv, 5);
  if (argc <= 0) {
    return;
  }

  if (argc == 1) {
    handleSingleTokenShortcut(argv[0]);
    return;
  }

  if (equalsIgnoreCase(argv[0], "help") || equalsIgnoreCase(argv[0], "h")) {
    printHelp();
    return;
  }
  if (equalsIgnoreCase(argv[0], "list")) {
    printAllStatus();
    return;
  }
  if (equalsIgnoreCase(argv[0], "use")) {
    handleUseCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "status")) {
    handleStatusCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "pos")) {
    handleSimpleTargetAction(argc, argv, "pos", readUnifiedPosition);
    return;
  }
  if (equalsIgnoreCase(argv[0], "zero")) {
    handleSimpleTargetAction(argc, argv, "zero", zeroUnifiedMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "on")) {
    handleSimpleTargetAction(argc, argv, "on", onUnifiedMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "off")) {
    handleSimpleTargetAction(argc, argv, "off", offUnifiedMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "stop")) {
    handleSimpleTargetAction(argc, argv, "stop", stopUnifiedMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "speed")) {
    handleSpeedCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "goto")) {
    handleGotoCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "g")) {
    handleGotoCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "t")) {
    if (!activeTargetIsOdrive()) {
      Serial.println("t <turns> is ODrive-only; select joint1, joint2, or joint3 first.");
      return;
    }
    if (argc != 2) {
      Serial.println("usage: t <turns>");
      return;
    }
    double relative_turns = 0.0;
    if (!parseDoubleToken(argv[1], &relative_turns)) {
      Serial.println("invalid turns value, expected something like: t 0.25");
      return;
    }
    NodeRuntimeState *state = odriveStateForNode(activeTarget().odrive_node_id);
    if (state == nullptr) {
      return;
    }
    if (!ensureOdriveReadyForMotion(activeTarget().odrive_node_id, "turn debug")) {
      return;
    }
    const float absolute_turns = state->zeroReferenceTurns + static_cast<float>(relative_turns);
    state->haveActiveTarget           = true;
    state->lastRelativeCommandTurns   = static_cast<float>(relative_turns);
    state->lastRelativeCommandDegrees =
        motorTurnsToJointDegrees(activeTarget().odrive_node_id, static_cast<float>(relative_turns));
    state->lastAbsoluteCommandTurns   = absolute_turns;
    state->lastTargetStreamMs         = millis();
    Serial.printf("%s relative turns=%.6f -> absolute turns=%.6f\n",
                  activeTarget().name,
                  relative_turns,
                  absolute_turns);
    sendOdriveInputPositionToNode(activeTarget().odrive_node_id, absolute_turns, 0, 0, "Sent Set_Input_Pos", true);
    return;
  }
  if (equalsIgnoreCase(argv[0], "raw")) {
    handleRawCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "telemetry")) {
    handleTelemetryCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "stream")) {
    handleStreamCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "mode")) {
    handleModeCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "osc")) {
    handleOscCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "selftest")) {
    sendSelfTestFrame();
    return;
  }

  Serial.printf("unknown command: %s\n", argv[0]);
  printHelp();
}

// ── Main serial input pump ────────────────────────────────────────────────────

inline void processSerialInput() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      g_serial_line[g_serial_line_len] = '\0';
      handleSerialLine(g_serial_line);
      g_serial_line_len   = 0;
      g_serial_line[0]    = '\0';
      continue;
    }
    if (g_serial_line_len + 1 >= sizeof(g_serial_line)) {
      Serial.println("serial command too long, clearing buffer");
      g_serial_line_len = 0;
      g_serial_line[0]  = '\0';
      continue;
    }
    g_serial_line[g_serial_line_len++] = ch;
    g_serial_line[g_serial_line_len]   = '\0';
  }
}
