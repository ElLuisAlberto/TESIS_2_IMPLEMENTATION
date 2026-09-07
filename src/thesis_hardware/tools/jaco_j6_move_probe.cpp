#include <Kinova.API.USBCommandLayerUbuntu.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr float kExpectedStartDeg = 63.37468719482422f;
constexpr float kExpectedTargetDeg = 58.37468719482423f;
constexpr float kDeltaDeg = -5.0f;
constexpr float kVelocityDegPerSec = kDeltaDeg / 10.0f;
constexpr double kDurationSec = 10.0;
constexpr float kStateToleranceDeg = 0.25f;

volatile std::sig_atomic_t g_stop_requested = 0;

void request_stop(int)
{
  g_stop_requested = 1;
}

double degrees_to_radians(float degrees)
{
  return static_cast<double>(degrees) * kPi / 180.0;
}

void initialize_zero_velocity_point(
  TrajectoryPoint & point,
  float j6_velocity_deg_per_sec)
{
  point.InitStruct();

  point.Position.Type = ANGULAR_VELOCITY;
  point.Position.HandMode = HAND_NOMOVEMENT;

  point.Position.Actuators.Actuator1 = 0.0f;
  point.Position.Actuators.Actuator2 = 0.0f;
  point.Position.Actuators.Actuator3 = 0.0f;
  point.Position.Actuators.Actuator4 = 0.0f;
  point.Position.Actuators.Actuator5 = 0.0f;
  point.Position.Actuators.Actuator6 = j6_velocity_deg_per_sec;
  point.Position.Actuators.Actuator7 = 0.0f;

  point.Position.Fingers.Finger1 = 0.0f;
  point.Position.Fingers.Finger2 = 0.0f;
  point.Position.Fingers.Finger3 = 0.0f;
}

bool parse_arguments(
  int argc,
  char ** argv,
  bool & execute,
  bool & confirmation)
{
  execute = false;
  confirmation = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);

    if (argument == "--help") {
      std::cout
        << "Uso:\n"
        << "  jaco_j6_move_probe                 # dry-run\n"
        << "  jaco_j6_move_probe --execute "
           "--confirm J6-5DEG-10S              # fisico\n";
      return false;
    }

    if (argument == "--execute") {
      execute = true;
      continue;
    }

    if (argument == "--confirm" && index + 1 < argc) {
      const std::string token(argv[++index]);
      confirmation = token == "J6-5DEG-10S";
      if (!confirmation) {
        std::cerr << "Confirmacion invalida; no se movera el brazo.\n";
        return false;
      }
      continue;
    }

    std::cerr << "Argumento no reconocido; no se movera el brazo.\n";
    return false;
  }

  if (confirmation && !execute) {
    std::cerr << "--confirm requiere --execute; no se movera el brazo.\n";
    return false;
  }

  if (execute && !confirmation) {
    std::cerr
      << "Falta --confirm J6-5DEG-10S; no se movera el brazo.\n";
    return false;
  }

  return true;
}

bool open_device(KinovaDevice & active_device)
{
  int result = InitAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "InitAPI fallo con codigo " << result << "\n";
    return false;
  }

  KinovaDevice devices[MAX_KINOVA_DEVICE];
  std::memset(devices, 0, sizeof(devices));

  int device_count = 0;
  result = GetDevices(devices, device_count);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "GetDevices fallo con codigo " << result << "\n";
    CloseAPI();
    return false;
  }

  if (device_count != 1) {
    std::cerr
      << "Se esperaba exactamente un JACO USB; encontrados: "
      << device_count << "\n";
    CloseAPI();
    return false;
  }

  active_device = devices[0];
  result = SetActiveDevice(active_device);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "SetActiveDevice fallo con codigo " << result << "\n";
    CloseAPI();
    return false;
  }

  return true;
}

bool read_current_position(AngularPosition & position)
{
  std::memset(&position, 0, sizeof(position));

  const int result = GetAngularPosition(position);
  if (result != NO_ERROR_KINOVA) {
    std::cerr
      << "GetAngularPosition fallo con codigo "
      << result << "\n";
    return false;
  }

  if (!std::isfinite(position.Actuators.Actuator6)) {
    std::cerr << "La posicion de J6 no es finita.\n";
    return false;
  }

  return true;
}

void close_api()
{
  const int result = CloseAPI();
  std::cout << "CloseAPI retorno " << result << "\n";
}

int send_zero_velocity_for_half_second()
{
  for (int index = 0; index < 100; ++index) {
    TrajectoryPoint zero_point;
    initialize_zero_velocity_point(zero_point, 0.0f);

    const int result = SendBasicTrajectory(zero_point);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  return NO_ERROR_KINOVA;
}

}  // namespace

int main(int argc, char ** argv)
{
  bool execute = false;
  bool confirmation = false;

  if (!parse_arguments(argc, argv, execute, confirmation)) {
    return 2;
  }

  KinovaDevice active_device;
  std::memset(&active_device, 0, sizeof(active_device));

  if (!open_device(active_device)) {
    return 1;
  }

  AngularPosition current_position;
  if (!read_current_position(current_position)) {
    close_api();
    return 1;
  }

  const float current_j6_deg = current_position.Actuators.Actuator6;
  const float target_j6_deg = current_j6_deg + kDeltaDeg;

  std::cout << std::fixed << std::setprecision(6)
            << "J6 actual:   " << current_j6_deg << " deg ("
            << degrees_to_radians(current_j6_deg) << " rad)\n"
            << "J6 objetivo: " << target_j6_deg << " deg ("
            << degrees_to_radians(target_j6_deg) << " rad)\n"
            << "Duracion:    " << kDurationSec << " s\n"
            << "Velocidad:   " << kVelocityDegPerSec << " deg/s\n"
            << "Dedos:       HAND_NOMOVEMENT\n";

  if (
    std::fabs(current_j6_deg - kExpectedStartDeg) >
    kStateToleranceDeg)
  {
    std::cerr
      << "El estado actual de J6 no coincide con el estado autorizado; "
         "no se movera el brazo.\n";
    close_api();
    return 1;
  }

  if (
    std::fabs(target_j6_deg - kExpectedTargetDeg) >
    kStateToleranceDeg)
  {
    std::cerr
      << "El objetivo calculado no coincide con el objetivo autorizado; "
         "no se movera el brazo.\n";
    close_api();
    return 1;
  }

  if (!execute) {
    std::cout
      << "DRY-RUN: no se llamaron StartControlAPI ni "
         "SendBasicTrajectory.\n";
    close_api();
    return 0;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  std::cerr
    << "\nADVERTENCIA: esta ejecucion movera fisicamente unicamente J6.\n"
    << "J6: " << current_j6_deg << " -> " << target_j6_deg
    << " grados en 10 s; dedos inmoviles.\n"
    << "Iniciando en 3 s. Presiona Ctrl+C para solicitar parada.\n";

  std::this_thread::sleep_for(std::chrono::seconds(3));

  int result = StartControlAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "StartControlAPI fallo con codigo " << result << "\n";
    close_api();
    return 1;
  }

  result = SetAngularControl();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "SetAngularControl fallo con codigo " << result << "\n";
    StopControlAPI();
    close_api();
    return 1;
  }

  TrajectoryPoint velocity_point;
  initialize_zero_velocity_point(
    velocity_point,
    kVelocityDegPerSec);
    
    
  const auto start_time = std::chrono::steady_clock::now();
  const auto end_time =
    start_time + std::chrono::milliseconds(15000);

  bool send_error = false;

  while (
    !g_stop_requested &&
    std::chrono::steady_clock::now() < end_time)
  {
    result = SendBasicTrajectory(velocity_point);

    if (result != NO_ERROR_KINOVA) {
      std::cerr
        << "SendBasicTrajectory fallo con codigo "
        << result << "\n";
      send_error = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (g_stop_requested) {
    std::cerr << "Parada solicitada; enviando velocidad cero.\n";
  }

  const int zero_result =
    send_zero_velocity_for_half_second();

  if (zero_result != NO_ERROR_KINOVA) {
    std::cerr
      << "Fallo al enviar velocidad cero: codigo "
      << zero_result << "\n";
    send_error = true;
  }

  const int stop_result = StopControlAPI();

  std::cout
    << "StopControlAPI retorno "
    << stop_result << "\n";

  AngularPosition final_position;

  if (read_current_position(final_position)) {
    std::cout << std::fixed << std::setprecision(6)
              << "J6 final: "
              << final_position.Actuators.Actuator6
              << " deg ("
              << degrees_to_radians(
                   final_position.Actuators.Actuator6)
              << " rad)\n";
  }

  close_api();

  return (send_error ||
          stop_result != NO_ERROR_KINOVA) ? 1 : 0;
}
    
