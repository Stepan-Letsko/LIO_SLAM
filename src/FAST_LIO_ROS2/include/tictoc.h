/**
 * @file tictoc.h
 * @brief A simple timer utility class for benchmarking code execution time.
 * 
 * This class wraps std::chrono to provide easy-to-use tic() and toc() methods
 * for measuring elapsed time in milliseconds. It is commonly used in SLAM
 * packages (like VINS-Mono, A-LOAM, FAST-LIO) for performance analysis.
 */

#pragma once
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <string>

class TicToc
{
  public:
    /**
     * @brief Constructor starts the timer immediately.
     */
    TicToc()
    {
        tic();
    }

    /**
     * @brief Resets the start time to the current moment.
     */
    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    /**
     * @brief Returns the elapsed time since the last tic() in milliseconds.
     * @return double Elapsed time in ms.
     */
    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
    
    /**
     * @brief Prints the elapsed time with a label to stdout.
     * @param about_task Description of the task being measured.
     */
    void toc(std::string about_task)
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout.precision(3); 
        std::cout << "[" << about_task << "] " << elapsed_seconds.count() * 1000 << " ms" << std::endl;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};