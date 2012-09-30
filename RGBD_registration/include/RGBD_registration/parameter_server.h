/*
 * parameter_server.h
 * 
 */
#ifndef PARAMETER_SERVER_H_
#define PARAMETER_SERVER_H_
#include <string>
#include <ros/ros.h>
#include <boost/any.hpp>

//this is a global definition of the points to be used
//changes to omit color would need adaptations in 
//the visualization too

/*!
 * \brief Getting values from parameter server.
 * This class is used for getting the parameters from
 * the parameter server
 */
class ParameterServer {
public:
    /*!
     * Returns the singleton instance
     */
    static ParameterServer* instance();

    /*!
     * The method returns a value from the local cache.
     * You can use bool, int, double and std::string for T
     *
     * \param param the name of the parameter
     * \return the parameter value
     */
    template<typename T>
    T get(const std::string param) {
        ROS_ERROR_COND(config.count(param)==0, "ParameterServer object queried for invalid parameter \"%s\"", param.c_str());
        boost::any value = config[param];
        return boost::any_cast<T>(value);
    }

private:
    std::map<std::string, boost::any> config;

    static ParameterServer* _instance;
    std::string pre;
    ros::NodeHandle handle;

    /*!
     * Default constructor
     * private, because of singleton
     */
    ParameterServer();

    /*!
     * Receives all values from the parameter server and store them
     * in the map 'config'.
     * Will be called in the constructor
     */
    void getValues();

    /*!
     * Loads the default configuration
     */
    void defaultConfig();

    /*!
     * Returns a value from the parameter server
     * Will only be used by getValue()
     *
     * \param param name of the parameter
     * \param def default value (get through defaultConfig())
     *
     * \return the parameter value
     */
    template<typename T>
    T getFromParameterServer(const std::string param, T def) {
        std::string fullParam = param;
        T result;
        handle.param(fullParam, result, def);
        return result;
    }
};

#endif /* PARAMETER_SERVER_H_ */
