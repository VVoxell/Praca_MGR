#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <cmath>
#include <vector>

class BasicDriver : public rclcpp::Node {
public:
	BasicDriver() : Node("driver"){
	RCLCPP_INFO(this->get_logger(), "Driver: ON");
	
	real_tag_positions_ = {
	  {3, {-1.5, 0.0}},
	  {4, {-1.5, 1.0}}
	};
	
	loadGraphPos("optimized.g2o");
    	computeTransformMatrix(3, 4);
  }
  
private:
	std::map<int, std::pair<float, float>> real_tag_positions_;
	std::map<int, std::pair<float, float>> graph_tag_positions_;
	
	float k = 1.0;
	float theta = 0.0;
	float tx = 0.0;
	float ty = 0.0;
	
	void loadGraphPos(const std::string& filepath) {
	  std::ifstream file(filepath);
	  if (!file.is_open()) {
	    RCLCPP_ERROR(this->get_logger(), "Nie udało się otworzyć pliku: %s", filepath.c_str());
	    return;
	  }
	  
	std::string line;
	while (std::getline(file, line)) {
	  if (line.rfind("VERTEX_XY", 0) == 0) {
	    std::istringstream iss(line);
	    std::string prefix;
	    int id;
	    float x, y;
	    iss >> prefix >> id >> x >> y;
	    graph_tag_positions_[id] = {x, y};
	    RCLCPP_INFO(this->get_logger(), "Wczytano tag ID %d z pozycji x=%.2f, y=%.2f", id, x, y);
	  }
	}
	
	file.close();
	}
	
	float computeDistance(std::pair<float, float> a, std::pair<float, float> b) {
	  return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
	}
  	
  	void computeTransformMatrix(int id1, int id2) {
  	
  	  if (!(graph_tag_positions_.count(id1) && graph_tag_positions_.count(id2))) {
  	    RCLCPP_WARN(this->get_logger(), "Brakuje tagów w grafie.");
  	    return;
    	  } 
    	  
    	  if (!(real_tag_positions_.count(id1) && real_tag_positions_.count(id2))) {
    	    RCLCPP_WARN(this->get_logger(), "Brakuje tagów w rzeczywistości.");
    	    return;
    	  } 
  	
  	  // Pozycje tagów (graf i rzeczywistość)
	  auto g1 = graph_tag_positions_[id1];
      	  auto g2 = graph_tag_positions_[id2];
	  auto r1 = real_tag_positions_[id1];
	  auto r2 = real_tag_positions_[id2];
	  
	  // Wektory różnic
  	  std::vector<float> graph{g2.first - g1.first, g2.second - g1.second}; 
  	  std::vector<float> real{r2.first - r1.first, r2.second - r1.second};
  	  
  	  // Odległości
  	  float graph_dist = computeDistance(g1, g2);
  	  float real_dist = computeDistance(r1, r2);
  	  k = real_dist / graph_dist; 		// współczynnik skali
  	  
  	  theta = std::atan2(real[1], real[0]) - std::atan2(graph[1], graph[0]); // kąt obrotu
  	  
  	  // Sinus i Cosinus thety
  	  float cos_t = std::cos(theta);
  	  float sin_t = std::sin(theta);
  	  
  	  //Translacja
  	  tx = r1.first - k * (cos_t * g1.first - sin_t * g1.second);
  	  ty = r1.second - k * (sin_t * g1.first + cos_t * g1.second);
  	  
	  
    	  RCLCPP_INFO(this->get_logger(), "Utworzono Transformacje między grafem a rzeczywistością:");
  	  RCLCPP_INFO(this->get_logger(), "Skala k = %.4f", k);
  	  RCLCPP_INFO(this->get_logger(), "Kąt theta = %.4f rad", theta);
  	  RCLCPP_INFO(this->get_logger(), "Translacja: dx = %.4f, dy = %.4f", tx, ty);
  	  
  	}
  	  
  	std::pair<float, float> applyTransform(float x, float y, bool inverse = false){
  	
  	  if(!inverse){
  	    float cos_t = std::cos(theta);
  	    float sin_t = std::sin(theta);
  	    
  	    float x_new = k * (cos_t * x - sin_t * y) + tx;
  	    float y_new = k * (sin_t * x + cos_t * y) + ty;
  	    return {x_new, y_new};
  	  } 
  	  else {
  	    float cos_t = std::cos(-theta);
  	    float sin_t = std::sin(-theta);
  	    float x_shift = x - tx;
  	    float y_shift = y - ty;
  	    
  	    float x_new = 1/k * (cos_t * x_shift - sin_t * y_shift);
  	    float y_new = 1/k * (sin_t * x_shift + cos_t * y_shift);
  	    return {x_new, y_new};
  	  }
  	}
  	
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicDriver>());
  rclcpp::shutdown();
  return 0;
}
