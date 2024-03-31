#include <iostream>
#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"
#include "path_optimizer.hpp" 
#include <fstream>
#include <numeric>
#include <vector>
#include "path_planner.hpp"
#include <thread>


int main()
{
  bool threading = false;
  std::cout << "Ray Casting and Diffusion model for Path Planning \n";



  if (threading)
  {
    std::string prefix = "/home/michael/github/rcd_path_planner/maps/random_boxes1/";

    std::ofstream outfile("/home/michael/github/rcd_path_planner/maps/random_boxes1/result_boxes/home_pc_results/rcd.csv");
    if (!outfile) {
        std::cerr << "Error: Unable to open file: "  << std::endl;
    }
    outfile << "coverage,time_mean,length_mean" << std::endl;

    for (int i = 0 ; i < 90 ; ++i)
    {
      std::string filename = prefix + std::to_string(i) +".ppm";
      PathPlanner rcd_path_planner(5, filename);

      std::vector<std::thread> threads;
    
      std::vector<float> scales = {3, 4, 5, 6, 10}; // Add as many values as needed

      for (int scale : scales) {
        threads.emplace_back([&rcd_path_planner, scale]() {
            rcd_path_planner.FindPath(scale);
        });
      }

      for (std::thread &thread : threads) {
        thread.join();
      }
      outfile << rcd_path_planner.bestThreadResult[0]<< ',' << rcd_path_planner.bestThreadResult[1] << ',' << rcd_path_planner.bestThreadResult[2] << std::endl;

    }
  }
  else{
    bool runAll = false;

    if (runAll){
      std::ofstream outfile("/home/michael/github/rcd_path_planner/maps/random_boxes1/result_boxes/home_pc_results/rcd.csv");
      if (!outfile) {
          std::cerr << "Error: Unable to open file: "  << std::endl;
      }
      outfile << "coverage,time_mean,length_mean" << std::endl;
      for (int i = 0 ; i < 90 ; ++i)
      {
        std::string prefix = "/home/michael/github/rcd_path_planner/maps/random_boxes1/";



        std::string filename = prefix + std::to_string(i) +".ppm";
        PathPlanner rcd_path_planner(50, filename);

        // PathPlanner rcd_path_planner(1, "/home/michael/github/rcd_path_planner/maps/random_boxes/1.ppm");

        rcd_path_planner.FindPath(10);

        outfile << rcd_path_planner.bestThreadResult[0]<< ',' << rcd_path_planner.bestThreadResult[1] << ',' << rcd_path_planner.bestThreadResult[2] << std::endl;

      }
    }else{
        PathPlanner rcd_path_planner(1,  "/home/michael/github/rcd_path_planner/maps/random_boxes1/0.ppm");
        rcd_path_planner.FindPath(6);


    }
  
  }
 




  return 0;
}
// #include "path_optimizer.hpp"
// #include "path_planner.hpp"

// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <thread>
// #include <mutex>
// #include <condition_variable>
// #include <queue>
// #include <functional>

// class ThreadPool {
// public:
//     ThreadPool(size_t num_threads) : stop(false) {
//         for (size_t i = 0; i < num_threads; ++i)
//             threads.emplace_back(std::bind(&ThreadPool::worker_thread, this));
//     }

//     ~ThreadPool() {
//         {
//             std::unique_lock<std::mutex> lock(queue_mutex);
//             stop = true;
//         }
//         condition.notify_all();
//         for (std::thread &worker : threads)
//             worker.join();
//     }

//     template<class F>
//     void submit(F&& f) {
//         {
//             std::unique_lock<std::mutex> lock(queue_mutex);
//             tasks.emplace(std::forward<F>(f));
//         }
//         condition.notify_one();
//     }

// private:
//     std::vector<std::thread> threads;
//     std::queue<std::function<void()>> tasks;
//     std::mutex queue_mutex;
//     std::condition_variable condition;
//     bool stop;

//     void worker_thread() {
//         while (true) {
//             std::function<void()> task;
//             {
//                 std::unique_lock<std::mutex> lock(queue_mutex);
//                 condition.wait(lock, [this] { return stop || !tasks.empty(); });
//                 if (stop && tasks.empty())
//                     return;
//                 task = std::move(tasks.front());
//                 tasks.pop();
//             }
//             task();
//         }
//     }
// };

// void process_scale(PathPlanner& rcd_path_planner, float scale, std::ofstream& outfile) {
//     rcd_path_planner.FindPath(scale);
//     outfile << rcd_path_planner.bestThreadResult[0] << ',' << rcd_path_planner.bestThreadResult[1] << ',' << rcd_path_planner.bestThreadResult[2] << std::endl;
// }

// int main() {
//     bool threading = false;
//     std::cout << "Ray Casting and Diffusion model for Path Planning \n";

//     if (threading) {
//         std::string prefix = "/home/michael/github/rcd_path_planner/maps/random_boxes1/";
//         std::ofstream outfile("/home/michael/github/rcd_path_planner/maps/random_boxes1/result_boxes/home_pc_results/rcd.csv");
//         if (!outfile) {
//             std::cerr << "Error: Unable to open file: " << std::endl;
//             return 1;
//         }
//         outfile << "coverage,time_mean,length_mean" << std::endl;

//         ThreadPool pool(std::thread::hardware_concurrency());
//         for (int i = 0; i < 90; ++i) {
//             std::string filename = prefix + std::to_string(i) + ".ppm";
//             PathPlanner rcd_path_planner(5, filename);

//             std::vector<float> scales = {3, 4, 6, 8, 10}; // Add as many values as needed

//             for (float scale : scales) {
//                 pool.submit([&rcd_path_planner, scale, &outfile]() {
//                     process_scale(rcd_path_planner, scale, outfile);
//                 });
//             }
//         }
//     } else {
//       bool runAll = false;

//       if (runAll){
//         std::ofstream outfile("/home/michael/github/rcd_path_planner/maps/random_boxes/results_boxes/home_pc_results/rcd.csv");
//         if (!outfile) {
//             std::cerr << "Error: Unable to open file: "  << std::endl;
//         }
//         outfile << "coverage,time_mean,length_mean" << std::endl;
//         for (int i = 0 ; i < 90 ; ++i)
//         {
//           std::string prefix = "/home/michael/github/rcd_path_planner/maps/random_boxes/";



//           std::string filename = prefix + std::to_string(i) +".ppm";
//           PathPlanner rcd_path_planner(1, filename);

//           // PathPlanner rcd_path_planner(1, "/home/michael/github/rcd_path_planner/maps/random_boxes/1.ppm");

//           rcd_path_planner.FindPath(6);

//           outfile << rcd_path_planner.bestThreadResult[0]<< ',' << rcd_path_planner.bestThreadResult[1] << ',' << rcd_path_planner.bestThreadResult[2] << std::endl;

//         }
//       }else{
//           PathPlanner rcd_path_planner(1,  "/home/michael/github/rcd_path_planner/maps/random_boxes1/0.ppm");
//           rcd_path_planner.FindPath(4);


//       }
//     }

//     return 0;
// }