#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <map>
#include <string>

// Estrutura para armazenar a configuração (cor e número de ciclos)
struct Config {
  std::string cor;
  uint8_t n;
};

class LedController : public rclcpp::Node {
public:
  LedController() : Node("led_controller"), current_n_(0), led_on_(false)
  {
    // Criação do subscriber para o tópico "signal" com std_msgs::msg::String
    signal_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "signal", 10,
      std::bind(&LedController::signal_callback, this, std::placeholders::_1));

    // Inicializa o mapa com as configurações
    config_map_ = {
      {"perigo_geral", {"vermelho", 1}},
      {"estrada_irregular", {"vermelho", 2}},
      {"animais_via", {"vermelho", 3}},
      {"estra_afunila", {"vermelho", 4}},
      {"parque", {"azul", 1}},
      {"vel_60", {"azul", 2}},
      {"hospital", {"azul", 2}},
      {"peoes", {"azul", 4}},
      {"esquerda", {"verde", 1}},
      {"luzes", {"verde", 1}},
      {"rotunda", {"verde", 2}},
      {"autocarro", {"verde", 4}},
    };

    RCLCPP_INFO(this->get_logger(), "Nó led_controller iniciado. Aguardando sinal...");
  }

private:
  void signal_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Procura a descrição no mapa de configurações
    auto search = config_map_.find(msg->data);
    if (search != config_map_.end()) {
      // Atualiza os parâmetros conforme a configuração encontrada
      current_color_ = search->second.cor;
      current_n_ = search->second.n;
      current_description_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Configuração encontrada para '%s': Cor: %s, n: %d",
                  current_description_.c_str(), current_color_.c_str(), current_n_);
      
      // Se houver um timer ativo de uma execução anterior, cancela-o
      if (timer_ != nullptr) {
        timer_->cancel();
      }
      
      // Inicia o ciclo de temporização se n > 0
      if (current_n_ > 0) {
        timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&LedController::timer_callback, this));
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Descrição desconhecida: '%s'", msg->data.c_str());
    }
  }

  void timer_callback()
  {
    if (current_n_ == 0) {
      // Finaliza o ciclo e cancela o timer
      timer_->cancel();
      timer_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Ciclo finalizado. Aguardando nova configuração.");
      return;
    }

    led_on_ = !led_on_;
    if (led_on_) {
      RCLCPP_INFO(this->get_logger(), "LED %s - ON - Descrição: %s (Ciclos restantes: %d)",
                  current_color_.c_str(), current_description_.c_str(), current_n_);
    } else {
      RCLCPP_INFO(this->get_logger(), "LED %s - OFF (Ciclos restantes: %d)",
                  current_color_.c_str(), current_n_);
      // Decrementa o contador após o ciclo ON+OFF
      current_n_--;
    }
  }

  // Variáveis do nó:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string current_color_;
  std::string current_description_;
  uint8_t current_n_;
  bool led_on_;

  // Mapa de configuração: descrição → (cor, n)
  std::map<std::string, Config> config_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedController>());
  rclcpp::shutdown();
  return 0;
}
