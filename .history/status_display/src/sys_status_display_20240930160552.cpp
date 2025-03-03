#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "customize_interface/msg/system_status.hpp"

using SystemStatus = customize_interface::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
public:
    SystemStatusDisplay() : Node("sys_status_display")
    {
        sub_ = this->create_subscription<SystemStatus>(
            "sys_status", 10, [&](const SystemStatus::SharedPtr msg) -> void
            { label_->setText(get_qstr_from_msg(msg)); });

        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    }
    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;
        show_str
            << "===========系统状态可视化显示工具============\n"
            << "Data Time:\t" << msg->stamp.sec << "\ts\n"
            << "Host Name:\t" << msg->host_name << "\t\n"
            << "CPU使用率:\t" << msg->cpu_percent << "\t%\n"
            << "RAM使用率:\t" << msg->memory_percent << "\t%\n"
            << "RAM總大小:\t" << msg->memory_total << "\tMB\n"
            << "剩餘有效RAM:\t" << msg->memory_available << "\tMB\n"
            << "網路發送量:\t" << msg->net_sent << "\tMB\n"
            << "網路接收量:\t" << msg->net_recv << "\tMB\n"
            << "==========================================";

        return QString::fromStdString(show_str.str());
    }

private:
    rclcpp::Subscription<SystemStatus>::SharedPtr sub_;
    QLabel *label_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&node]() -> void
                            { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();

    return 0;
}