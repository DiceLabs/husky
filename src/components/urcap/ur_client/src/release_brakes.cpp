#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client.h>

#include <iostream>
#include <memory>
#include <thread>

#include "release_brakes.h"

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string LEFT_ROBOT_IP = "192.168.131.40";
const std::string RIGHT_ROBOT_IP = "192.168.131.41";

std::unique_ptr<urcl::DashboardClient> setupDashboard(const std::string& ip)
{
    urcl::setLogLevel(urcl::LogLevel::DEBUG);
    return std::make_unique<urcl::DashboardClient>(ip);
}

std::unique_ptr<urcl::DashboardClient> connectClient(const std::string& robot_ip)
{
  const char* connect_error_msg = "Could not connect to dashboard";
  auto dashboard = setupDashboard(robot_ip);
  if (!dashboard->connect())
  {
    URCL_LOG_ERROR(connect_error_msg);
    throw std::runtime_error(connect_error_msg);
  }
  return std::move(dashboard);
}

void release_brakes()
{
    const auto left_dashboard = connectClient(LEFT_ROBOT_IP);
    const auto right_dashboard = connectClient(RIGHT_ROBOT_IP);

    left_dashboard->commandClosePopup();
    left_dashboard->commandPowerOn();
    left_dashboard->commandCloseSafetyPopup();
    left_dashboard->commandUnlockProtectiveStop();
    left_dashboard->commandBrakeRelease();

    right_dashboard->commandClosePopup();
    right_dashboard->commandPowerOn();
    right_dashboard->commandCloseSafetyPopup();
    right_dashboard->commandUnlockProtectiveStop();
    right_dashboard->commandBrakeRelease();
}

void unlockProtectiveStop()
{
    const auto left_dashboard = connectClient(LEFT_ROBOT_IP);
    const auto right_dashboard = connectClient(RIGHT_ROBOT_IP);
    left_dashboard->commandUnlockProtectiveStop();
    right_dashboard->commandUnlockProtectiveStop();
}

void pressPlayButton()
{
    const auto left_dashboard = connectClient(LEFT_ROBOT_IP);
    const auto right_dashboard = connectClient(RIGHT_ROBOT_IP);
    left_dashboard->commandPlay();
    right_dashboard->commandPlay();
}