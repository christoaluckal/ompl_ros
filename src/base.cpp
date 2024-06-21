#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <ompl/util/PPM.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Plane2DEnvironment
{
public:
    Plane2DEnvironment(const char *ppm_file ="", bool use_deterministic_sampling = false, int min_solutions_ = 3)
    {
        useDeterministicSampling_ = use_deterministic_sampling;
        min_solutions = min_solutions_;
        // check if ppm_file is "" or not
        if (ppm_file == "")
        {
            std::cout << "No file provided, set the map to plan" << std::endl;
            first_map_set = false;
        }
        else{
            first_map_set = true;
            resetMap(ppm_file, use_deterministic_sampling);
        }
    }


    void resetMap(const char *ppm_file, bool use_deterministic_sampling = false)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch (ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            auto space = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(ss_->getStateSpace());
            space->setBounds(0.0, ppm_.getWidth());
            space->setBounds(1.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            ss_->getStateSpace()->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            first_map_set = true;
        }
        if (useDeterministicSampling_)
        {
            // PRMstar can use the deterministic sampling
            ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!ss_)
            return false;
        if (!first_map_set)
        {
            std::cout << "No map set, cannot plan" << std::endl;
            return false;
        }
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;
        for (int i = 0; i < min_solutions; ++i)
        {
            if (ss_->getPlanner())
                ss_->getPlanner()->clear();
            ss_->solve();
        }
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath())
        {
            if (!useDeterministicSampling_)
                ss_->simplifySolution();

            og::PathGeometric &p = ss_->getSolutionPath();
            if (!useDeterministicSampling_)
            {
                ss_->getPathSimplifier()->simplifyMax(p);
                ss_->getPathSimplifier()->smoothBSpline(p);
            }

            return true;
        }

        return false;
    }

    std::vector<std::vector<double>> returnPath()
    {
        std::vector<std::vector<double>> path;
        if (!ss_ || !ss_->haveSolutionPath())
            return path;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); ++i)
        {
            std::vector<double> point;
            point.push_back(p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            point.push_back(p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            path.push_back(point);
        }
        return path;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h =
                std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
        std::cout << "Saved the result to '" << filename << "'" << std::endl;
    }

    void updateThreshold(double new_thresh)
    {
        threshold_ = new_thresh;
    }

private:
    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        double gray = 0.2126 * c.red + 0.7152 * c.green + 0.0722 * c.blue;
    //  return c.red > 127 && c.green > 127 && c.blue > 127;
    return gray < 127;
    }

    ob::StateSamplerPtr allocateHaltonStateSamplerRealVector(const ompl::base::StateSpace *space, unsigned int dim,
                                                            std::vector<unsigned int> bases = {})
    {
        // specify which deterministic sequence to use, here: HaltonSequence
        // optionally we can specify the bases used for generation (otherwise first dim prime numbers are used)
        if (bases.size() != 0)
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(bases.size(), bases));
        else
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(dim));
    }

    

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    int min_solutions;
    bool useDeterministicSampling_;
    double threshold_;
    bool first_map_set;
};

// int main(int /*argc*/, char ** /*argv*/)
// {
//     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

//     bool useDeterministicSampling = false;
//     std::string env_file = "/home/christoa/Developer/summer2024/ros2_ws/src/ompl_planner/src/landscape.ppm";
// //  Plane2DEnvironment env("/home/christoa/Developer/summer2024/3d_planning/py_scripts/landscape.ppm".c_str(), useDeterministicSampling);
// Plane2DEnvironment env(env_file.c_str(), useDeterministicSampling);

//     if (env.plan(0, 0, 80, 80))
//     {
//         env.recordSolution();
//         env.save("result_demo.ppm");
//     }

//     return 0;
// }