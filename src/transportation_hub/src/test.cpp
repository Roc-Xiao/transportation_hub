//
// Created by roc on 25-2-5.
//
#include "Judger.h"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <unordered_map>

using Question = judger_interfaces::msg::OverallInfo;
using Answer = judger_interfaces::msg::MyAnswer;
using Road = judger_interfaces::msg::RoadInfo;
using Server = judger_interfaces::srv::MyService;

class test : public rclcpp::Node
{
  public:
      test()
      : Node("minimal_subscriber")
      {
          subscription_ = this->create_subscription<Question>("question", 10, std::bind(&test::topic_callback, this, std::placeholders::_1));
      }

  private:
      void topic_callback(const Question & msg) const
      {
          //msg读取
          int number_of_cities = msg.number_of_cities;
          int number_of_roads = msg.number_of_roads;
          std::vector<Road> road_info = msg.road_infos; //？
          int src_city = msg.src_city;
          int des_city = msg.des_city;

          //邻接表
          std::unordered_map<int, std::vector<std::pair<int, int>>> graph;
          for (const auto &road : road_info) {
              graph[road.source].emplace_back(road.destination, road.length);
              graph[road.destination].emplace_back(road.source, road.length); //如果是无向图
          }

          //Dijkstra算法
          std::vector<int> shortest_path = dijkstra(graph, src_city, des_city);

          //构造答案
          Answer answer;
          answer.my_answer = shortest_path;

          //发送答案到服务端
          std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
          rclcpp::Client<Server>::SharedPtr client = node->create_client<Server>("judger_server");

          auto request = std::make_shared<Server::Request>();
          request->answer = answer;

          while (!client->wait_for_service(std::chrono::seconds(1))) {
              if (!rclcpp::ok()) {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the server");
                  return;
              }
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
          }

          auto result = client->async_send_request(request);
          if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Score: %d", result.get()->score);
          } else {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service judger_server");
          }
      }

      std::vector<int> dijkstra(const std::unordered_map<int, std::vector<std::pair<int, int>>> &graph, int src, int des) const
      {
          //优先存储队列 (当前距离, 当前节点)
          std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

          //距离数组
          std::unordered_map<int, int> distances;
          for (const auto &city : graph) {
              distances[city.first] = std::numeric_limits<int>::max();
          }
          distances[src] = 0;

          //前驱节点数组
          std::unordered_map<int, int> predecessors;

          pq.emplace(0, src);

          while (!pq.empty()) {
              int current_dist = pq.top().first;
              int current_node = pq.top().second;
              pq.pop();

              //如果当前节点是目标节点，停止搜索
              if (current_node == des) {
                  break;
              }

              //如果当前距离大于记录的距离，跳过
              if (current_dist > distances[current_node]) {
                  continue;
              }

              //遍历邻接表
              for (const auto &neighbor : graph.at(current_node)) {
                  int neighbor_node = neighbor.first;
                  int edge_weight = neighbor.second;

                  //计算新距离
                  int new_dist = current_dist + edge_weight;

                  //如果新距离更短，更新距离和前驱节点
                  if (new_dist < distances[neighbor_node]) {
                      distances[neighbor_node] = new_dist;
                      predecessors[neighbor_node] = current_node;
                      pq.emplace(new_dist, neighbor_node);
                  }
              }
          }

          //回溯路径，在具体实现路径重现的时候，主要是借助前驱数组和栈来进行实现
          std::vector<int> path;
          int current_node = des;
          while (current_node != src) {
              path.push_back(current_node);
              current_node = predecessors[current_node];
          }
          path.push_back(src);
          std::reverse(path.begin(), path.end());

          return path;
      }

      rclcpp::Subscription<Question>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<test>());
    rclcpp::shutdown();

    return 0;
}