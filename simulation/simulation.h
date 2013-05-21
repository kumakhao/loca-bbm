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
//
//
#include "robot.h"
#include "camera.h"
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
	Simulation();
	~Simulation();
	void Initialize();
	void Realize();
	void Step();
	void setLocalisation(localisation *loca);
	void enableLandmarks();
	void enablePadControl();
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
	bool particles_on_;
	bool landmarks_on_;
	bool setup_done_;
	bool picture_processed_;
	bool pad_control_on_;
	bool take_picture_button_pressed_;

	double old_increments_right_;
	double old_increments_left_;
	double step_counter_;
	double view_matrix_distance_;

	int64 take_picture_timer_;

	osg::Group *robot_;
	osg::Group *root_;
	osg::Matrix view_matrix_;
//	osg::Matrixd projectionMatrix;
	osg::Vec3d view_matrix_eye_, view_matrix_center_, view_matrix_up_;
	osgViewer::Viewer viewer_;

	landmarkSet landmarks_;
	ScreenShotCallback *screen_shot_callback_;
	localisation *localisation_;
	Particles *particle_view_;
	DataWriter data_to_file_writer_;
	RobotData *robotdata_;
	cJoystick *sixaxes_;

};


#endif /* SIMULATION_H_ */
