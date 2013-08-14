/*
 * simulation.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */


#include "simulation.h"
#include <osgGA/TrackballManipulator>

Simulation::Simulation():
	particles_on_(false),
	setup_done_(false),
	picture_processed_(true),
	pad_control_on_(false),
	take_picture_button_pressed_(false),
	observe_mode_(noObserve),
	old_increments_right_(0.0),
	old_increments_left_(0.0),
	step_counter_(0.0),
	view_matrix_distance_(0.0),
	take_picture_timer_(0),
	loop_time_(0),
	loop_target_time_(33333),
	data_to_file_writer_("/home/josef/workspace/Loca-Projekt/pictures/","locaDatafile.txt"),
	sixaxes_(NULL)
{
	trajectory_from_file_.clear();
	//viewer_.setThreadingModel(osgViewer::CompositeViewer::ThreadingModel::SingleThreaded);
}

void Simulation::Initialize() {

	root_ = SetupScene();
	robot_ = SetupRobot();
	root_->addChild(robot_);
	root_->addChild(hud_.getGroup());
	screen_shot_callback_ = new ScreenShotCallback();
	root_->addUpdateCallback(new Particles::ParticleNodeCallback());
	osgViewer::View* view_robot = new osgViewer::View;
	osgViewer::View* view_map = new osgViewer::View;
	//osgViewer::Viewer::ThreadingModel test;
	std::cout<<"ThreadingModelbefore: "<<viewer_.getThreadingModel()<<std::endl;
	viewer_.setThreadingModel((osgViewer::ViewerBase::ThreadingModel) 0);
	std::cout<<"ThreadingModelafter: "<<viewer_.getThreadingModel()<<std::endl;
	viewer_.setUpThreading();

	viewer_.setUpThreading();
	viewer_.addView(view_robot);
	viewer_.addView(view_map);

	//TODO light test
	{
		mylightsource = new osg::LightSource();
		osg::Light *mylight = new osg::Light();
		mylight->setLightNum(0);
		mylightsource->setLight(mylight);
		root_->addChild(mylightsource);
		mylightsource->setStateSetModes(*root_->getOrCreateStateSet(), osg::StateAttribute::ON);
		mylight->setAmbient(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
		mylight->setDiffuse(osg::Vec4d(1.0, 1.0, 1.0, 1.0));
		mylight->setPosition(osg::Vec4d(0.0, 0.0, 10.0, 1.0));
	}

	if(particles_on_){
		particle_view_ = new Particles();
		particle_view_->Populate(localisation_->param.nrOfParticles);
		localisation_->createSamples(localisation_->param.nrOfParticles);
//			particle_view_->Populate(1);
//			localisation_->createOneParticle();


		particle_view_->Update(localisation_->getParticles());
		particle_view_->AddToThis(root_);

		std::vector<int> landmark_IDs;
		switch (observe_mode_){
		case Landmarks:
			// Setup visualisation of landmarks.
			landmark_IDs = landmarks_.getIDVector();
			for(std::vector<int>::iterator it = landmark_IDs.begin();it != landmark_IDs.end();it++){
				osg::PositionAttitudeTransform* landmark_view = new osg::PositionAttitudeTransform();
				landmark_view->addChild(locaUtil::pyramid());
				landmark_view->setScale(osg::Vec3d(0.1,0.1,0.1));
				landmark_view->setPosition( osg::Vec3d(landmarks_.getXofID(*it),landmarks_.getYofID(*it),0.0) );
				root_->addChild(landmark_view);
			}
			break;

		default:
			break;
		}

	}

	if(pad_control_on_)
		sixaxes_ = new cJoystick();
	if( (!pad_control_on_) || (!sixaxes_->isActiv()) ){
		KeyboardEventHandler* robotControlHandler;
		robotControlHandler = new KeyboardEventHandler((RobotData*)robot_->getUserData());
		view_robot->addEventHandler(robotControlHandler);
		pad_control_on_ = false;
	}


	{
		view_robot->setSceneData(root_);
		view_robot->setUpViewInWindow(10,10,800,600);
		osg::ref_ptr<osgGA::NodeTrackerManipulator> robot_camera_manipulator = new osgGA::NodeTrackerManipulator;
		robot_camera_manipulator->setHomePosition(osg::Vec3(0, -3, 0), osg::Vec3(0,0,0), osg::Vec3(0,-1.0,0));
		robot_camera_manipulator->setTrackNode(robot_->getChild(0));
		robot_camera_manipulator->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
		view_robot->setCameraManipulator(robot_camera_manipulator);
		view_robot->getCamera()->setFinalDrawCallback(screen_shot_callback_);
	}

    {
    	view_map->setSceneData(root_);
		view_map->setUpViewInWindow(820,10,320,320);
		osg::ref_ptr<osgGA::NodeTrackerManipulator> map_camera_manipulator = new osgGA::NodeTrackerManipulator;
		//osg::ref_ptr<osgGA::TrackballManipulator> trackball_mani = new osgGA::TrackballManipulator;
		map_camera_manipulator->setHomePosition(osg::Vec3(0, 0, 23), osg::Vec3(0,0,0), osg::Vec3(0,0,0));
		map_camera_manipulator->setTrackNode(robot_->getChild(0));
		map_camera_manipulator->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
		//view_fix->getCameraManipulator()->setHomePosition(osg::Vec3(0, -3, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,0));
		view_map->setCameraManipulator( map_camera_manipulator );
    }


	// attach a trackball manipulator to all user control of the view
//	osgGA::TrackballManipulator *trackballMani = new osgGA::TrackballManipulator;
//	trackballMani->setHomePosition(osg::Vec3(0, -3, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,0));
//	viewer_.setCameraManipulator(trackballMani);
	//viewer_.getCamera()->setViewMatrix(osg::Matrix::lookAt(osg::Vec3(0, -30, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,0)));

	// A manipulator to follow the robot node.
//	osg::ref_ptr<osgGA::NodeTrackerManipulator> manipulator = new osgGA::NodeTrackerManipulator;
//	manipulator->setHomePosition(osg::Vec3(0, -3, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,0));
//	manipulator->setTrackNode(robot_->getChild(0));
//	manipulator->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
//	viewer_.setCameraManipulator(manipulator);


}

void Simulation::Realize() {

	// create the windows and start the required threads.
	viewer_.realize();
	take_picture_timer_ = cvGetTickCount();
	loop_time_ = take_picture_timer_;
	//osg::Matrix MVPW = viewMatrix*projectionMatrix*windowMatrix;
	setup_done_ = true;
}

void Simulation::Step() {
	// dispatch the new frame, this wraps up the follow Viewer operations:
	//   advance() to the new frame
	//   eventTraversal() that collects events and passes them on to the event handlers and event callbacks
	//   updateTraversal() to calls the update callbacks
	//   renderingTraversals() that runs syncronizes all the rendering threads (if any) and dispatch cull, draw and swap buffers

	int64 tmptime = cvGetTickCount();
	int64 difftime = (tmptime-loop_time_)/cvGetTickFrequency();
	if(difftime < loop_target_time_){
		std::cout<<"sleeping: "<<loop_target_time_-difftime<<std::endl;
		usleep(loop_target_time_-difftime);
		return;
	}
	loop_time_ = tmptime;

	viewer_.advance();
	viewer_.eventTraversal();
	viewer_.updateTraversal();
	viewer_.renderingTraversals();

	step_counter_ ++;
	robotdata_ = dynamic_cast<RobotData*> (robot_->getUserData());

	viewer_.getView(0)->getCamera()->getViewMatrixAsLookAt(view_matrix_eye_, view_matrix_center_, view_matrix_up_, view_matrix_distance_);
	view_matrix_ = viewer_.getView(0)->getCamera()->getViewMatrix();
//	osg::Matrix windowMatrix = viewer_.getView(0)->getCamera()->getViewport()->computeWindowMatrix();
//	osg::Matrix projectionMatrix = viewer_.getView(0)->getCamera()->getProjectionMatrix();
//	osg::Matrix mat = projectionMatrix*windowMatrix;
//	std::cout<<"mat: "<<mat(0,0)<<" "<<mat(1,1)<<" "<<mat(2,0)<<" "<<mat(2,1)<<std::endl;
	std::cout<<"eye: "<<view_matrix_eye_.x()<<" "<<view_matrix_eye_.y()<<" "<<view_matrix_eye_.z()<<std::endl;

	//Picture handling
	if(screen_shot_callback_->isPicTaken() && !picture_processed_){
		osg::ref_ptr<osg::Image> osgImage = screen_shot_callback_->getImage();
		cv::Mat cvImg(osgImage->t(), osgImage->s(), CV_8UC3);
		cv::Mat cvCopyImg(osgImage->t(), osgImage->s(), CV_8UC3);
		cvImg.data = (uchar*)osgImage->data();
		cv::flip(cvImg, cvCopyImg, 0); // Flipping because of different origins
		observedImg_ = &cvCopyImg;

		// Write position, orientation and image to log file.
		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
												robotdata_->x_pos_, view_matrix_eye_[0], localisation_->getPosition().at(0),
												robotdata_->y_pos_, view_matrix_eye_[1], localisation_->getPosition().at(2),
												robotdata_->psi_, view_matrix_(0,0), localisation_->getOrientation().at(1), cvCopyImg);
		Observe();
		//		old version
		//		data_to_file_writer_.WritePlotData(	robotdata_->x_pos_, robotdata_->y_pos_, robotdata_->psi_,
		//											localisation_->getPosition().at(0), localisation_->getPosition().at(2),
		//											localisation_->getPosition().at(1)*3, localisation_->getPosition().at(3)*3, true);
				localisation::EstimatedRobotPose es = localisation_->getEstimatedRobotPose();
				data_to_file_writer_.WritePlotData(	robotdata_->x_pos_, robotdata_->y_pos_,
													es.x, es.y,
													es.sigmaXYLarge, es.sigmaXYSmall,
													es.sigmaXYAngle/M_PI*180, true);
		// observe for particle filter is done here.
		picture_processed_ = true;
	}

	else if(step_counter_ > 0){
		// This block is only executed every 20th pass.
		step_counter_ = 0;
		if(particles_on_){
			// The dynamic update of the particle filter.
			localisation_->dynamic(robotdata_->incremente_left_-old_increments_left_,robotdata_->incremente_right_-old_increments_right_);

			// Update of simulations view of particles.
			particle_view_->Update(localisation_->getParticles());
		}
		// writes the current position and orientation of the robot to file.
		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
														robotdata_->x_pos_, view_matrix_eye_[0], localisation_->getPosition().at(0),
														robotdata_->y_pos_, view_matrix_eye_[1], localisation_->getPosition().at(2),
														robotdata_->psi_, view_matrix_(0,0), localisation_->getOrientation().at(1));
//		old version
//		data_to_file_writer_.WritePlotData(	robotdata_->x_pos_, robotdata_->y_pos_, robotdata_->psi_,
//											localisation_->getPosition().at(0), localisation_->getPosition().at(2),
//											localisation_->getPosition().at(1)*3, localisation_->getPosition().at(3)*3);
		localisation::EstimatedRobotPose es = localisation_->getEstimatedRobotPose();
		data_to_file_writer_.WritePlotData(	robotdata_->x_pos_, robotdata_->y_pos_,
											es.x, es.y,
											es.sigmaXYLarge, es.sigmaXYSmall,
											es.sigmaXYAngle/M_PI*180);
		old_increments_left_ = robotdata_->incremente_left_;
		old_increments_right_ = robotdata_->incremente_right_;
	}

	UpdateHUD();

	// without pad_control pictures are taken every 2sec
	if(!pad_control_on_ && (0.001*(cvGetTickCount()-take_picture_timer_)/cvGetTickFrequency()>2000) ){
		screen_shot_callback_->queueShot();
		picture_processed_ = false;
		take_picture_timer_ = cvGetTickCount();
	}
	if(pad_control_on_){
		if(picture_processed_ && !take_picture_button_pressed_ && sixaxes_->buttonPressed(BUTTON_CIRCLE)){
				// Every 2 sec a Shot is queued to get a picture form the scene.
				screen_shot_callback_->queueShot();
				take_picture_button_pressed_ = true;
				picture_processed_ = false;
				take_picture_timer_ = cvGetTickCount();
		}

		if(!sixaxes_->buttonPressed(BUTTON_CIRCLE))
			take_picture_button_pressed_ = false;

		// If Padcontrol is enabled, the inputs from the pad are written to the robotData.
		// That way the callback will use the inputs to move the robot.
		if(sixaxes_->buttonPressed(BUTTON_L1) && sixaxes_->buttonPressed(BUTTON_R1)){
			robotdata_->speed_ = sixaxes_->axis(AXIS_LY)*-0.2;
			robotdata_->psi_speed_ = sixaxes_->axis(AXIS_RX)*-0.05;
		}
		else{
			robotdata_->speed_ = 0.0;
			robotdata_->psi_speed_ = 0.0;
		}
		if(sixaxes_->buttonPressed(BUTTON_TRIANGLE)){
			while(viewer_.areThreadsRunning()){
				viewer_.stopThreading();
			}
			viewer_.setDone(true);
		}
	}

	if(!trajectory_from_file_.empty()){
		double temp = trajectory_from_file_.front();
		if(temp<1){
			screen_shot_callback_->queueShot();
			picture_processed_ = false;
		}
		trajectory_from_file_.erase(trajectory_from_file_.begin());

		temp = trajectory_from_file_.front();
		if(temp>0){
			while(viewer_.areThreadsRunning()){
				viewer_.stopThreading();
			}
			viewer_.setDone(true);
		}
		trajectory_from_file_.erase(trajectory_from_file_.begin());

		temp = trajectory_from_file_.front();
		robotdata_->speed_ = temp;
		trajectory_from_file_.erase(trajectory_from_file_.begin());

		temp = trajectory_from_file_.front();
		robotdata_->psi_speed_ = temp;
		trajectory_from_file_.erase(trajectory_from_file_.begin());
	}
	else{
		trajectory_buffer_ << screen_shot_callback_->isPicTaken() << std::endl;
		trajectory_buffer_ << viewer_.done() << std::endl;
		trajectory_buffer_ << robotdata_->speed_ << std::endl;
		trajectory_buffer_ << robotdata_->psi_speed_ << std::endl;
	}
	//trajectory_buffer_ << "------------------" << std::endl;
}

double Simulation::getRobX() {
	return robotdata_->x_pos_;
}

double Simulation::getRobY() {
	return robotdata_->y_pos_;
}

double Simulation::getRobPsi() {
	return robotdata_->psi_;
}

double Simulation::getLandmarkObservation(int ID) {
	return 	landmarks_.getAngleToLandmark(ID, robotdata_->x_pos_, robotdata_->y_pos_, robotdata_->psi_);
}

double Simulation::getLeftInc() {
	double value = robotdata_->incremente_left_-old_increments_left_;
	old_increments_left_ = robotdata_->incremente_left_;
	return value;
}

void Simulation::setLocalisation(localisation* loca) {
	particles_on_ = true;
	this->localisation_ = loca;
}

void Simulation::setObserveMode(ObserveMode mode) {
	switch (mode){
	case noObserve:
		observe_mode_ = mode;
		break;

	case GPS:
		observe_mode_ = mode;
		break;

	case Landmarks:
		if(localisation_){
			this->landmarks_ = localisation_->landmarks;
			observe_mode_ = mode;
		}
		break;

	case Pictures:
		observe_mode_ = mode;
		break;

	default:
		break;
	}
}

double Simulation::getRightInc() {
	double value = robotdata_->incremente_right_-old_increments_right_;
	old_increments_right_ = robotdata_->incremente_right_;
	return value;
}

void Simulation::enablePadControl() {
	this->pad_control_on_ = true;
}

Simulation::~Simulation() {
	if(sixaxes_)
		delete sixaxes_;
}

bool Simulation::done() {
	return this->viewer_.done();
}

osg::Geode* Simulation::Leinwand() {
	osg::Geode* leinwandGeode = new osg::Geode();
	osg::Geometry* leinwandGeometry = new osg::Geometry();
	leinwandGeode->addDrawable(leinwandGeometry);

	// Specify the vertices:
	osg::Vec3Array* leinwandVertices = new osg::Vec3Array;
	leinwandVertices->push_back( osg::Vec3(0, 0, 0) ); // front left
	leinwandVertices->push_back( osg::Vec3(8, 0, 0) ); // front right
	leinwandVertices->push_back( osg::Vec3(8, 0, 3) ); // top right
	leinwandVertices->push_back( osg::Vec3(0, 0, 3) ); // top left

	leinwandGeometry->setVertexArray( leinwandVertices );

	osg::DrawElementsUInt* leinwand1 =
			new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
	leinwand1->push_back(3);
	leinwand1->push_back(2);
	leinwand1->push_back(1);
	leinwand1->push_back(0);
	leinwandGeometry->addPrimitiveSet(leinwand1);

	osg::Vec2Array* texcoords = new osg::Vec2Array(5);
	(*texcoords)[0].set(0.0f,0.0f); // tex coord for vertex 0
	(*texcoords)[1].set(1.0f,0.0f); // tex coord for vertex 1
	(*texcoords)[2].set(1.0f,1.0f); // ""
	(*texcoords)[3].set(0.0f,1.0f); // ""
	leinwandGeometry->setTexCoordArray(0,texcoords);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	leinwandGeometry->setColorArray(colors);
	leinwandGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	return leinwandGeode;
}

osg::Geode* Simulation::Ground() {
	osg::Geode* GroundGeode = new osg::Geode();
	osg::Geometry* GroundGeometry = new osg::Geometry();
	GroundGeode->addDrawable(GroundGeometry);

	// Specify the vertices:
	osg::Vec3Array* GroundVertices = new osg::Vec3Array;
	GroundVertices->push_back( osg::Vec3(-6, -6, 0) ); // bottom left
	GroundVertices->push_back( osg::Vec3(6, -6, 0) ); // bottom right
	GroundVertices->push_back( osg::Vec3(6, 6, 0) ); // top right
	GroundVertices->push_back( osg::Vec3(-6, 6, 0) ); // top left

	GroundGeometry->setVertexArray( GroundVertices );

	osg::DrawElementsUInt* Ground =
			new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
	Ground->push_back(3);
	Ground->push_back(2);
	Ground->push_back(1);
	Ground->push_back(0);
	GroundGeometry->addPrimitiveSet(Ground);

	osg::Vec2Array* texcoords = new osg::Vec2Array(5);
	(*texcoords)[0].set(0.0f,0.0f); // tex coord for vertex 0
	(*texcoords)[1].set(1.0f,0.0f); // tex coord for vertex 1
	(*texcoords)[2].set(1.0f,1.0f); // ""
	(*texcoords)[3].set(0.0f,1.0f); // ""
	GroundGeometry->setTexCoordArray(0,texcoords);

	return GroundGeode;
}

osg::Group* Simulation::SetupScene() {
	osg::Node* ground = Ground();
	osg::Node* leinwand_01 = Leinwand();
	osg::Node* leinwand_02 = Leinwand();
	osg::Node* leinwand_03 = Leinwand();
	// Declare a node which will serve as the root node
	// for the scene graph. Since we will be adding nodes
	// as 'children' of this node we need to make it a 'group'
	// instance.
	// The 'node' class represents the most generic version of nodes.
	// This includes nodes that do not have children (leaf nodes.)
	// The 'group' class is a specialized version of the node class.
	// It adds functions associated with adding and manipulating
	// children.

	osg::Group* root = new osg::Group();
	root->addChild(ground);
	osg::PositionAttitudeTransform* leinwand01_transform = new osg::PositionAttitudeTransform();
	osg::PositionAttitudeTransform* leinwand02_transform = new osg::PositionAttitudeTransform();
	osg::PositionAttitudeTransform* leinwand03_transform = new osg::PositionAttitudeTransform();
	leinwand01_transform->setPosition(osg::Vec3(6,6,0));
	leinwand02_transform->setPosition(osg::Vec3(-6,6,0));
	leinwand03_transform->setPosition(osg::Vec3(6,-6,0));
	leinwand03_transform->setAttitude(osg::Quat(M_PI,osg::Vec3d(0.0,0.0,1.0)));
	leinwand01_transform->setAttitude(osg::Quat(-M_PI/2,osg::Vec3d(0.0,0.0,1.0)));
	root->addChild(leinwand01_transform);
	root->addChild(leinwand02_transform);
	root->addChild(leinwand03_transform);
	leinwand01_transform->addChild(leinwand_01);
	leinwand02_transform->addChild(leinwand_02);
	leinwand03_transform->addChild(leinwand_03);

	osg::Texture2D* leinwandFaceTexture01 = new osg::Texture2D;
	osg::Texture2D* leinwandFaceTexture02 = new osg::Texture2D;
	osg::Texture2D* leinwandFaceTexture03 = new osg::Texture2D;

	// protect from being optimized away as static state:
	leinwandFaceTexture01->setDataVariance(osg::Object::DYNAMIC);
	leinwandFaceTexture02->setDataVariance(osg::Object::DYNAMIC);
	leinwandFaceTexture03->setDataVariance(osg::Object::DYNAMIC);

	// load an image by reading a file:
	osg::Image* leinwandFace01 = osgDB::readImageFile("textures/wall1_1024.jpg");
	osg::Image* leinwandFace02 = osgDB::readImageFile("textures/wall2_1024.jpg");
	osg::Image* leinwandFace03 = osgDB::readImageFile("textures/wall3_1024.jpg");
//		osg::Image* leinwandFace01 = osgDB::readImageFile("/home/josef/workspace/test/Checkerboard_pattern_01.png");
//		osg::Image* leinwandFace02 = osgDB::readImageFile("/home/josef/workspace/test/Checkerboard_pattern_02.png");
//		osg::Image* leinwandFace03 = osgDB::readImageFile("/home/josef/workspace/test/Checkerboard_pattern_03.png");
	if (!leinwandFace01) std::cout << " couldn't find texture." << std::endl;
	if (!leinwandFace02) std::cout << " couldn't find texture." << std::endl;
	if (!leinwandFace03) std::cout << " couldn't find texture." << std::endl;

	// Assign the texture to the image we read from file:
	leinwandFaceTexture01->setImage(leinwandFace01);
	leinwandFaceTexture02->setImage(leinwandFace02);
	leinwandFaceTexture03->setImage(leinwandFace03);

	// Create a new StateSet with default settings:
	osg::StateSet* state_leinwand01 = new osg::StateSet();
	osg::StateSet* state_leinwand02 = new osg::StateSet();
	osg::StateSet* state_leinwand03 = new osg::StateSet();

	// Assign texture unit 0 of our new StateSet to the texture
	// we just created and enable the texture.
	state_leinwand01->setTextureAttributeAndModes
			  (0,leinwandFaceTexture01,osg::StateAttribute::ON);
	state_leinwand02->setTextureAttributeAndModes
			  (0,leinwandFaceTexture02,osg::StateAttribute::ON);
	state_leinwand03->setTextureAttributeAndModes
			  (0,leinwandFaceTexture03,osg::StateAttribute::ON);

	// Associate this state set with the Geode.
	leinwand01_transform->setStateSet(state_leinwand01);
	leinwand02_transform->setStateSet(state_leinwand02);
	leinwand03_transform->setStateSet(state_leinwand03);

/*osg::PositionAttitudeTransform *lightPos = new osg::PositionAttitudeTransform();

	osg::LightSource* testLight = new osg::LightSource();
	osg::StateSet *lightStateSet;
	lightStateSet = root->getOrCreateStateSet();
	testLight->setLight(createLight(1));
	testLight->setLocalStateSetModes(osg::StateAttribute::ON);
	testLight->setStateSetModes(*lightStateSet, osg::StateAttribute::ON);

	lightPos->addChild(testLight);
	lightPos->setPosition(osg::Vec3(-6, 0, 4));
	root->addChild(lightPos);*/
	return root;
}

void Simulation::Observe() {
	if(!particles_on_)
		return;
	std::vector<int> landmark_IDs;
	switch (observe_mode_){
	case GPS:
		localisation_->observeGPS(robotdata_->x_pos_,robotdata_->y_pos_,robotdata_->psi_);
		localisation_->resample(localisation_->param.nrOfParticles);
		break;

	case Landmarks:
		landmark_IDs = landmarks_.getIDVector();
		for(std::vector<int>::iterator it = landmark_IDs.begin();it != landmark_IDs.end();it++)
			localisation_->observeLandmark(*it, landmarks_.getAngleToLandmark(*it,robotdata_->x_pos_,robotdata_->y_pos_,robotdata_->psi_));
		localisation_->resample(localisation_->param.nrOfParticles);
		break;
	case Pictures:
		localisation_->observeImg(observedImg_);
		localisation_->resample(localisation_->param.nrOfParticles);
		break;
	default:
		break;
	}
}

void Simulation::CleanUp() {
	data_to_file_writer_.SaveImages();
}

void Simulation::Dynamic() {
}

void Simulation::ReadRobotTrajectory(std::string path) {
	std::ifstream datafile;
	std::string line;
	datafile.open(path.c_str(), std::ios_base::in);
	if(datafile.is_open()){
		trajectory_from_file_.clear();
		while(datafile.good()){
			getline(datafile,line);
			trajectory_from_file_.push_back(std::atof(line.c_str()));
		}
		datafile.close();
	}
}

void Simulation::UpdateHUD() {
	std::stringstream hud_text;
	//TODO: getPosition() is still used here, I might need to change to getEstimatedRobotPose()
	hud_text <<    "True Position: (x "<<robotdata_->x_pos_<<
								" / y "<<robotdata_->y_pos_<<
									" )"<<std::endl;
	hud_text <<    "Part Position: (x "<<this->localisation_->getPosition().at(0)<<
								" / y "<<this->localisation_->getPosition().at(2)<<
									" )"<<std::endl;
	hud_text <<    "VAR  Position: (x "<<this->localisation_->getPosition().at(1)<<
								" / y "<<this->localisation_->getPosition().at(3)<<
									" )"<<std::endl;
	hud_text <<    "Nr of Particles: "<<localisation_->param.nrOfParticles<<"("<<localisation_->good_rating_count<<")"<<
					"   Observe Highscore: "<<localisation_->highscore<<"("<<localisation_->highscore_count<<")";
	hud_.setText(hud_text.str());
}

void Simulation::WriteRobotTrajectory(std::string path) {
	if(trajectory_buffer_.str().length() == 0){
		std::cout<<"abord writeTrajectory"<<std::endl;
		return;
	}
	std::ofstream datafile;
	datafile.open (path.c_str(), std::ios_base::out);
	datafile << trajectory_buffer_.str();
	datafile.close();
}





