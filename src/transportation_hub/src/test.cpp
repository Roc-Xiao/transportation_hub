//
// Created by roc on 25-2-5.
//
#include <rclcpp/rclcpp.hpp>
#include <judger_interfaces/msg/overall_info.hpp>
#include <judger_interfaces/msg/road_info.hpp>
#include <judger_interfaces/srv/my_service.hpp>
#include "dijkstra.h"

using Question = judger_interfaces::msg::OverallInfo;
using Road = judger_interfaces::msg::RoadInfo;
using Server = judger_interfaces::srv::MyService;

class TestSubscriber : public rclcpp::Node {
public:
    TestSubscriber() : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<Question>(
            "question", 10,
            std::bind(&TestSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const Question::SharedPtr msg) const {
        int number_of_cities = msg->number_of_cities;
        int number_of_roads = msg->number_of_roads;
        const auto& road_info = msg->road_infos;
        int src_city = msg->src_city;
        int des_city = msg->des_city;

        unordered_map<int, vector<pair<int, int>>> graph;
        for (const auto& road : road_info) {
            graph[road.source].emplace_back(road.destination, road.length);
            graph[road.destination].emplace_back(road.source, road.length);
        }

        vector<int> shortest_path = dijkstra(graph, src_city, des_city);

        // 构造答案并发送（假设已实现 Answer 和服务调用）
    }

    rclcpp::Subscription<Question>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestSubscriber>());
    rclcpp::shutdown();
    return 0;
}