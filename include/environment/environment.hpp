#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>

#include "environment/directions.hpp"
#include "environment/field.hpp"
#include "parser/parser.hpp"

namespace vbd {

// Environment simulator
class Environment {
public:
  /*!
   * Constructor.
   * @brief Initialize an Environment.
   */
  explicit Environment(Config &config);
  /*!
   * @brief Generate a random new Environment from parsed config settings.
   * Overwrites previously generated Environment.
   */
  void generateNewEnvironmentFromSettings();
    /*!
   * @brief Generate a custom new Environment by filling up points inbetween corners given in the objectList
   * Overwrites previously generated Environment.
   */
  void generateCustomEnvironmentFromSettings();
  /*!
   * @brief Loads image data using SFML. Overwrites previously generated
   * Environment.
   * @param [in] filename Filename.
   */
  bool loadImage(const std::string &filename);

  void loadMaps(const std::string &filename);
  std::vector<float> stringToFloatVector(const std::string &str,
                                         const char delimiter);

  // Get visibility field shared pointer.
  inline const auto &getOccupancyField() const {
    return sharedOccupancyField_;
  }
  // Get parsed configuration.
  inline const auto &getConfig() const { return sharedConfig_; };

  // Deconstructor
  ~Environment() = default;

  // VBM
  // Get visibility field shared pointer.
  inline const auto &getVisibilityField() const {
    return sharedVisibilityField_;
  };
  // Get speed field shared pointer.
  inline const auto &getSpeedField() const { return sharedSpeedField_; };

private:
  size_t ny_;
  size_t nx_;
  size_t size_;
  int seedValue_ = 1;

  // Shared pointer to a Field object of type int that has map occupancy data
  std::shared_ptr<Field<int>> sharedOccupancyField_;
  // Shared pointer to configuration
  std::shared_ptr<Config> sharedConfig_;
  // Unique pointer to image holder
  std::unique_ptr<sf::Image> uniqueLoadedImage_;

  void saveEnvironment();
  void resetEnvironment();

  // VBM
   double speedValue_ = 2.0;
  // Shared pointer to a Field object of type double that has map occupancy
  // values.
  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Field<double>> sharedSpeedField_;
};

} // namespace vbd
#endif // ENVIRONMENT_HPP
