/*
 * simulation.h
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <osgGA/NodeTrackerManipulator>
#include <opencv.hpp>
#include <osg/Node>
#include <osg/Texture2D>
#include <osgViewer/CompositeViewer>

//
//
#include "robot.h"
#include "camera.h"
#include "hud.h"
#include "screenShotCallback.h"
#include "particles.h"
#include "../locaUtil.h"
#include "../unitTests.h"
#include "dataToFileWriter.h"
#include "../localisation/cameraParam.h"
#include "../util/cjoystick.h"
#include "../util/SIXAXIS_Interface.h"

class Simulation{
public:
	enum ObserveMode
	{
		noObserve,
		GPS,
		Landmarks,
		Pictures,
		OneParticle
	};
	class Settings{
	public:
		//Where the file for plots is stored.
		std::string plotfile_;
		//Where pictures, that have been used for Localization, are stored.
		std::string picture_path_;
		//Where the basic datafile is stored.
		std::string datafile_name_;
		//Time between pictures, when no Pad is used.
		int takepicture_intervall_;
		//Minimum time for one frame loop
		int loop_target_time_;
		//how many human models for the simulation are created
		int crowd_size_;
		//how many particels are visable in the simulation 1=1:1, 2=1:2, 3=1:3 usw
		int particle_visibility_ratio_;
		//enable or disable systematic errors
		bool sys_error_on_;
		//Camera picture width
		int camera_picture_width_;
		//Camera picture height
		int camera_picture_height_;
		//Camera field of View horizontal in degrees
		double field_of_view_horizontal_;
		//Disable windows
		bool blind_mode_on_;

		//Robot hardware paramerters including sensor nois!
		RobotData::RobotParameter robParameter_;

		Settings():
		picture_path_("/home/josef/workspace/Loca-Projekt/pictures/"),
		datafile_name_("locaDatafile.txt"),
		takepicture_intervall_(3000),
		loop_target_time_(33333),
		crowd_size_(0),
		particle_visibility_ratio_(1),
		sys_error_on_(true),
		camera_picture_width_(1280),
		camera_picture_height_(720),
		field_of_view_horizontal_(74),
		blind_mode_on_(false)
		{
			std::ostringstream filename;
			time_t t = time(0);   // get time now
			struct tm * now = localtime( & t );
			filename << "/home/josef/workspace/Loca-Projekt/plots/"
				<< (now->tm_year + 1900) << '-'
				<< std::setw(2) << std::setfill('0') <<(now->tm_mon + 1) << '-'
				<< std::setw(2) << std::setfill('0') << now->tm_mday << '_'
				<< std::setw(2) << std::setfill('0') << now->tm_hour
				<< std::setw(2) << std::setfill('0') << now->tm_min;
			plotfile_ = filename.str();

			robParameter_.kDistanceLeftWheel		= 0.35;
			robParameter_.kDistanceRightWheel		= 0.35;
			robParameter_.kLeftWheelWidth			= 0.05;
			robParameter_.kRightWheelWidth			= 0.05;
			robParameter_.kRadiusWheelRight			= 0.08;
			robParameter_.kRadiusWheelLeft			= 0.08;
			robParameter_.kImpulesProMeter			= 57694;
			robParameter_.kImpulePerTurn			= 2000;
			robParameter_.kTransmissionKoefficent	= 14.5;
			robParameter_.kPsiSpeed					= 0.05;
			robParameter_.kSigmaIncrement			= 0.05;
			robParameter_.kSpeed					= 0.1;
		}
	};
	// Controles how the simulation behaves
	// will only be applyed during Simulation::Initialize()
	Settings settings_;
	Simulation();
	~Simulation();
	void Initialize();
	void Realize();
	void Step();
	void CleanUp();
	void setLocalisation(localisation *loca);
	void setObserveMode(ObserveMode mode);
	void enablePadControl();
	void WriteRobotTrajectory(std::string path);
	void WriteRobotTrajectory();
	void ReadRobotTrajectory(std::string path);
	double getRobX();
	double getRobY();
	double getRobPsi();
	double getLandmarkObservation(int ID);
	double getLeftInc();
	double getRightInc();
	bool done();

private:
	osg::Geode* Leinwand();
	osg::Geode* Ground();
	osg::Group* SetupScene();
	osg::Geode* HumanDouble();
	void Observe();
	void Dynamic();
	void UpdateHUD();
	void AddCrowd(osg::Group *root_, int size);

	bool particles_on_;
	bool setup_done_;
	bool picture_processed_;
	bool pad_control_on_;
	bool take_picture_button_pressed_;
	bool blind_mode_on_;

	ObserveMode observe_mode_;


	double view_matrix_distance_;
	double field_of_view_horizontal_;

	int step_counter_;
	int takepicture_intervall_;
	int loop_target_time_;
	int croud_size_;
	int particle_visibility_ratio_;
	int camera_picture_width_;
	int camera_picture_height_;


	std::stringstream trajectory_buffer_;
	std::vector<double> trajectory_from_file_;
	int64 take_picture_timer_;
	int64 loop_time_;

	osg::Group *robot_;
	osg::Group *root_;
	osg::Matrix view_matrix_;
//	osg::Matrixd projectionMatrix;
	osg::Vec3d view_matrix_eye_, view_matrix_center_, view_matrix_up_;
	osgViewer::CompositeViewer viewer_;
	osg::LightSource *mylightsource;
	cv::Mat *observedImg_;

	landmarkSet landmarks_;
	HUD hud_;
	ScreenShotCallback *screen_shot_callback_;
	localisation *localisation_;
	Particles *particle_view_;
	DataWriter data_to_file_writer_;
	RobotData *robotdata_;
	cJoystick *sixaxes_;
	localisation::EstimatedRobotPose* es_;
};


#endif /* SIMULATION_H_ */
