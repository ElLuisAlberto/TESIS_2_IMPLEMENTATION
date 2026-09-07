#include <iostream>
#include <string>
#include "Kinova.API.USBCommandLayerUbuntu.h"

bool check(const char* operation, int result)
{
    std::cout << operation << ": " << result << std::endl;
    return result == NO_ERROR_KINOVA;
}

struct ApiSession
{
    ~ApiSession()
    {
        check("CloseAPI", CloseAPI());
    }
};

int main(int argc, char** argv)
{
    if (argc != 2 || std::string(argv[1]) != "--read-state")
    {
        std::cout << "Sin conexion al robot. Para consultar se requiere "
                     "el argumento --read-state.\n";
        return 0;
    }

    if (!check("InitAPI", InitAPI()))
        return 1;

    ApiSession session;

    KinovaDevice devices[MAX_KINOVA_DEVICE] = {};
    int result = 0;
    const int count = GetDevices(devices, result);

    if (!check("GetDevices", result))
        return 1;

    std::cout << "Dispositivos encontrados: " << count << std::endl;
    if (count != 1)
    {
        std::cerr << "Se requiere exactamente un dispositivo; "
                     "no se seleccionara ninguno.\n";
        return 1;
    }

    if (!check("SetActiveDevice", SetActiveDevice(devices[0])))
        return 1;

    AngularPosition position = {};
    if (!check("GetAngularPosition", GetAngularPosition(position)))
        return 1;

    const auto& a = position.Actuators;
    std::cout << "Posiciones originales de la API, sin conversion:\n"
              << "A1: " << a.Actuator1 << "\n"
              << "A2: " << a.Actuator2 << "\n"
              << "A3: " << a.Actuator3 << "\n"
              << "A4: " << a.Actuator4 << "\n"
              << "A5: " << a.Actuator5 << "\n"
              << "A6: " << a.Actuator6 << std::endl;
    return 0;
}
