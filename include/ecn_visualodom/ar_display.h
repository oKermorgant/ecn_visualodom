#ifndef AR_DISPLAY_H
#define AR_DISPLAY_H

#include <visp/vpAROgre.h>
#include <visp/vpImage.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageConvert.h>
#include <visp/vpTime.h>
#include <sstream>

class ModelDisplay
{
public:
    ModelDisplay() {init_ = false;}

    void init(const vpCameraParameters &_cam, vpImage<vpRGBa> &_I)
    {
        init_ = true;
        // find current ROS distro
        std::string ros_path = "/opt/ros/indigo/";
        if(!vpIoTools::checkDirectory(ros_path))
            ros_path = "/opt/ros/kinetic/";

        // find current ViSP installation
        std::vector<int> v = {0,1,2,3,4,5,6,7,8,9};
        std::string ogre_path = "";
        for(const auto &i: v)
        {
            for(const auto &j: v)
            {
                for(const auto &k: v)
                {
                    std::stringstream ss;
                    ss << ros_path << "share/visp-" << i << "." << j << "." << k << "/data/ogre-simulator/resources.cfg";
                   if(vpIoTools::checkFilename(ss.str()))
                    {
                        ogre_path = ss.str();
                        ogre_path.resize(ogre_path.size()-13);
                        break;
                    }
                }
                if(ogre_path.size())
                    break;
            }
            if(ogre_path.size())
                break;
        }
        std::cout << "Loading vpAROgre from:" << std::endl;
        std::cout << " - ROS: " << ros_path + "lib/visp/data/ogre-simulator/" << std::endl;
        std::cout << " - Meshes: " << ogre_path << std::endl;

        ogre_ =    vpAROgre(_cam, _I.getWidth(), _I.getHeight(), ogre_path.c_str(), (ros_path + "lib/visp/data/ogre-simulator/").c_str());
        ogre_.init(_I);
        ogre_.load("Robot", "robot.mesh");
        ogre_.setScale("Robot", 0.002f,0.002f,0.002f);
        ogre_.setRotation("Robot", vpRotationMatrix(vpRxyzVector(-M_PI/2, -M_PI/2, 0)));
        // Add an optional point light source
        Ogre::Light * light = ogre_.getSceneManager()->createLight();
        light->setDiffuseColour(1, 1, 1); // scaled RGB values
        light->setSpecularColour(1, 1, 1); // scaled RGB values
        light->setPosition(5, 5, -10);
        light->setType(Ogre::Light::LT_POINT);
    }

    void init(const vpCameraParameters &_cam, cv::Mat &_im)
    {
        vpImageConvert::convert(_im, I_);
        init(_cam, I_);
    }

    ModelDisplay(const vpCameraParameters &_cam, vpImage<vpRGBa> &_I)
    {
        init(_cam, _I);
    }

    // with opencv inputs
    ModelDisplay(const vpCameraParameters &_cam, cv::Mat &_im)
    {
        init(_cam, _im);
    }


    inline bool continueRendering()
    {
        if(!init_) return true;
        return ogre_.continueRendering();
    }

    inline void display(const vpImage<vpRGBa> &_I, const vpHomogeneousMatrix &_cMw)
    {
        if(!init_) return;
        ogre_.display(_I, _cMw);
        vpTime::wait(15);
    }


    inline void display(cv::Mat &_im, const vpHomogeneousMatrix &_cMw)
    {
        vpImageConvert::convert(_im, I_);
        display(I_, _cMw);
    }

protected:
    vpAROgre ogre_;
    vpImage<vpRGBa> I_;
    bool init_;
};



#endif // AR_DISPLAY_H
