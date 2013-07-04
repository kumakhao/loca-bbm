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
		Pictures
	};
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
	void Observe();
	void Dynamic();
	void UpdateHUD();
	bool particles_on_;
	bool setup_done_;
	bool picture_processed_;
	bool pad_control_on_;
	bool take_picture_button_pressed_;

	ObserveMode observe_mode_;

	double old_increments_right_;
	double old_increments_left_;
	double step_counter_;
	double view_matrix_distance_;

	std::stringstream trajectory_buffer_;
	std::vector<double> trajectory_from_file_;
	int64 take_picture_timer_;

	osg::Group *robot_;
	osg::Group *root_;
	osg::Matrix view_matrix_;
//	osg::Matrixd projectionMatrix;
	osg::Vec3d view_matrix_eye_, view_matrix_center_, view_matrix_up_;
	osgViewer::CompositeViewer viewer_;

	cv::Mat *observedImg_;

	landmarkSet landmarks_;
	HUD hud_;
	ScreenShotCallback *screen_shot_callback_;
	localisation *localisation_;
	Particles *particle_view_;
	DataWriter data_to_file_writer_;
	RobotData *robotdata_;
	cJoystick *sixaxes_;
};


#endif /* SIMULATION_H_ */
