#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/util/PPM.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;


std::vector<double> mean_path_list;
std::vector<double> mean_time_list;

void statistics(const std::vector<double> &vec, double &mean, double &std_dev) {
    // Compute mean (average of elements)
    double sum = 0.0;
    for (size_t i = 0; i < vec.size(); i++) {
        sum += vec[i];
    }
    mean = sum / (double)vec.size();

    // Compute sum squared differences with mean.
    double sqDiff = 0.0;
    for (size_t i = 0; i < vec.size(); i++) {
        sqDiff += (vec[i] - mean) * (vec[i] - mean);
    }
    std_dev = std::sqrt(sqDiff / (double)vec.size());
}

class Planner2D {
   public:

    Planner2D(const char *ppm_file, size_t N, const std::string &type) {
        N_ = N;
        bool map_loaded = false;
        try {
            ppm_.loadFile(ppm_file);
            map_loaded = true;
        } catch (ompl::Exception &ex) {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (map_loaded) {
            auto space(std::make_shared<ob::RealVectorStateSpace>());
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            ss_ = std::make_shared<og::SimpleSetup>(space);

            // set state validity checking for this space
            ss_->setStateValidityChecker(
                [this](const ob::State *state) { return isStateValid(state); });
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
                1.0 / space->getMaximumExtent());

            // set the Planner
            std::cout << "Running " << type << " planner" << std::endl;
            if (type == "RRTConnect") {
                ss_->setPlanner(std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
            } else if (type == "RRTstar") {
                ss_->setPlanner(std::make_shared<og::RRTstar>(ss_->getSpaceInformation()));
            } else if (type == "RRT") {
                ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
            } else if (type == "KPIECE1") {
                ss_->setPlanner(std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
            } else if (type == "LBKPIECE1") {
                ss_->setPlanner(std::make_shared<og::LBKPIECE1>(ss_->getSpaceInformation()));
            } else if (type == "SBL") {
                ss_->setPlanner(std::make_shared<og::SBL>(ss_->getSpaceInformation()));
            } else {
                ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
            }

            ss_->getProblemDefinition()->setOptimizationObjective(
                std::make_shared<ompl::base::PathLengthOptimizationObjective>(
                    ss_->getSpaceInformation()));
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row,
              unsigned int goal_col) {
        if (!ss_) {
            return false;
        }
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);

        // Generate a few solutions
        std::vector<double> times;
        std::vector<double> lengths;
        times.reserve(N_);
        lengths.reserve(N_);
        for (size_t i = 0; i < N_; i++) {
            if (ss_->getPlanner()) {
                ss_->getPlanner()->clear();
            }
            std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
            start_time = std::chrono::system_clock::now();
            ss_->solve();
            end_time = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end_time - start_time;
            OMPL_INFORM("Time %f", elapsed_seconds.count());
            times.push_back(elapsed_seconds.count());
            if (ss_->haveSolutionPath()) {
                ss_->simplifySolution();
                auto path = ss_->getSolutionPath();
                lengths.push_back(path.length());
                OMPL_INFORM("Path length %f", path.length());
            }
        }

        double time_mean{}, time_std{}, length_mean{}, length_std{};
        statistics(times, time_mean, time_std);
        statistics(lengths, length_mean, length_std);
        OMPL_INFORM("Mean time %f, time std %f ", time_mean, time_std);
        OMPL_INFORM("Mean path length %f, path length std %f ", length_mean, length_std);
        mean_path_list.push_back(length_mean);
        mean_time_list.push_back(time_mean);

        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath()) {
            ss_->simplifySolution();
            auto path = ss_->getSolutionPath();
            OMPL_INFORM("Path length %f", path.length());
            return true;
        }
        return false;
    }

    void recordSolution() {
        if (!ss_ || !ss_->haveSolutionPath()) {
            return;
        }
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); i++) {
            const int w =
                std::min(maxWidth_,
                         (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h =
                std::min(maxHeight_,
                         (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 0;
            c.green = 255;
            c.blue = 0;
        }
    }

    void save(const char *filename) {
        if (!ss_) {
            return;
        }
        ppm_.saveFile(filename);
    }

   private:
    bool isStateValid(const ob::State *state) const {
        const int w =
            std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h =
            std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    size_t N_;
};

int main(int argc, char *argv[]) {
    // Example run:
    // ./ompl_benchmark planner_runs planner_type path_to_map initial_position target_position
    //
    // ./ompl_benchmark 10 "SBL" "/path/to/0.ppm" 465 290 765 435
    size_t planner_runs;
    std::string planner_type;
    std::string path_to_map;
    size_t initial_position[2];
    size_t target_position[2];
    if (argc < 3) {
        std::cout << "Not enough arguments passed " << argc << std::endl;
        return 1;
    } else {
        planner_runs = std::atoi(argv[1]);
        planner_type = std::string(argv[2]);
    }

    std::string prefix = "/home/michael/github/rcd_path_planner/maps/mazes2/";
    
    for (int i = 0 ; i < 100 ; ++i)
    {   
      
      path_to_map = prefix + std::to_string(i) + ".ppm";
      auto path_to_yaml = prefix + std::to_string(i) +".yaml";
      YAML::Node config = YAML::LoadFile(path_to_yaml);

      initial_position[0] = config["robot_position_x"].as<size_t>();
      initial_position[1] = config["robot_position_y"].as<size_t>();
      target_position[0]  = config["target_position_x"].as<size_t>();
      target_position[1]  = config["target_position_y"].as<size_t>();


      std::cout << initial_position[0] << '\n';


      // Initialize planner
      Planner2D planner(path_to_map.c_str(), planner_runs, planner_type);

          // Run the planner and overlay the optimal trajectory to the map
      if (planner.plan(initial_position[0], initial_position[1], target_position[0],
                       target_position[1])) {
          // planner.recordSolution();
          // planner.save("result_demo.ppm");
      }

    }


  float time_mean = std::accumulate(mean_time_list.begin(), mean_time_list.end(), 0.0f) / mean_time_list.size();
  float path_length_mean = std::accumulate(mean_path_list.begin(), mean_path_list.end(), 0.0f) / mean_path_list.size();
 
  // Compute standard deviation of time vector
  float time_std = 0.0f;
  for (float t : mean_time_list) {
      time_std += (t - time_mean) * (t - time_mean);
  }
  time_std = std::sqrt(time_std / mean_time_list.size());

// Compute standard deviation of path_length vector
  float path_length_std = 0.0f;
  for (float pl : mean_path_list) {
      path_length_std += (pl - path_length_mean) * (pl - path_length_mean);
  }
  path_length_std = std::sqrt(path_length_std / mean_path_list.size());

// Print results
  std::cout << "Mean time: " << time_mean ;
  std::cout << "   std time : " << time_std << std::endl;
  std::cout << "Mean of path length: " << path_length_mean;
  std::cout << "   std of path length: " << path_length_std << std::endl;

    


    return 0;
}
