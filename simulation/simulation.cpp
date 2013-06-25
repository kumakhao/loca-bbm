/*
 * simulation.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */


#include "simulation.h"


Simulation::Simulation():
	particles_on_(false),
	setup_done_(false),
	picture_processed_(false),
	pad_control_on_(false),
	take_picture_button_pressed_(false),
	observe_mode_(noObserve),
	old_increments_right_(0.0),
	old_increments_left_(0.0),
	step_counter_(0.0),
	view_matrix_distance_(0.0),
	take_picture_timer_(0),
	data_to_file_writer_("/home/josef/workspace/Loca-Projekt/pictures/","locaDatafile.txt"),
	sixaxes_(NULL)
{
}

void Simulation::Initialize() {

	root_ = SetupScene();
	robot_ = SetupRobot();
	root_->addChild(robot_);
	screen_shot_callback_ = new ScreenShotCallback();
	root_->addUpdateCallback(new Particles::ParticleNodeCallback());

	if(particles_on_){
		particle_view_ = new Particles();
//		particle_view_->Populate(localisation_->param.nrOfParticles);
//		localisation_->createSamples(localisation_->param.nrOfParticles);
			particle_view_->Populate(1);
			localisation_->createOneParticle();


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

	// Declare a 'viewer'
	if(pad_control_on_)
		sixaxes_ = new cJoystick();
	if( (!pad_control_on_) || (!sixaxes_->isActiv()) ){
		KeyboardEventHandler* robotControlHandler;
		robotControlHandler = new KeyboardEventHandler((RobotData*)robot_->getUserData());
		viewer_.addEventHandler(robotControlHandler);
		pad_control_on_ = false;
	}


	viewer_.getCamera()->setFinalDrawCallback(screen_shot_callback_);
	viewer_.setSceneData( root_ );

	// attach a trackball manipulator to all user control of the view
	//viewer.setCameraManipulator(new osgGA::TrackballManipulator);

	// A manipulator to follow the robot node.
	osg::ref_ptr<osgGA::NodeTrackerManipulator> manipulator = new osgGA::NodeTrackerManipulator;
	viewer_.setCameraManipulator(manipulator);
	manipulator->setHomePosition(osg::Vec3(0, -3, 0), osg::Vec3(0,0,0), osg::Vec3(0,0,0));
	manipulator->setTrackNode(robot_->getChild(0));
	manipulator->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);

}

void Simulation::Realize() {

	// create the windows and start the required threads.
	viewer_.realize();
	take_picture_timer_ = cvGetTickCount();

	//osg::Matrix MVPW = viewMatrix*projectionMatrix*windowMatrix;
	setup_done_ = true;
}

void Simulation::Step() {
	// dispatch the new frame, this wraps up the follow Viewer operations:
	//   advance() to the new frame
	//   eventTraversal() that collects events and passes them on to the event handlers and event callbacks
	//   updateTraversal() to calls the update callbacks
	//   renderingTraversals() that runs syncronizes all the rendering threads (if any) and dispatch cull, draw and swap buffers
	viewer_.frame();
	step_counter_ ++;
	robotdata_ = dynamic_cast<RobotData*> (robot_->getUserData());

	viewer_.getCamera()->getViewMatrixAsLookAt(view_matrix_eye_, view_matrix_center_, view_matrix_up_, view_matrix_distance_);
	view_matrix_ = viewer_.getCamera()->getViewMatrix();
//	osg::Matrix windowMatrix = viewer_.getCamera()->getViewport()->computeWindowMatrix();
//	osg::Matrix projectionMatrix = viewer_.getCamera()->getProjectionMatrix();
//	osg::Matrix mat = projectionMatrix*windowMatrix;
//	std::cout<<"mat: "<<mat(0,0)<<" "<<mat(1,1)<<" "<<mat(2,0)<<" "<<mat(2,1)<<std::endl;
	//Picture handling
	if(screen_shot_callback_->isPicTaken() && !picture_processed_){
		osg::ref_ptr<osg::Image> osgImage = screen_shot_callback_->getImage();
		cv::Mat cvImg(osgImage->t(), osgImage->s(), CV_8UC3);
		cvImg.data = (uchar*)osgImage->data();
		cv::flip(cvImg, cvImg, 0); // Flipping because of different origins

		localisation_->observeImg(&cvImg);
		// Write position, orientation and image to log file.
		//dataWriter->writeData(robotData->incrementeLeft, robotData->incrementeRight, robotData->posX,robotData->posY,robotData->psi, cvImg);
		//data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_, view_matrix_eye_[0],view_matrix_eye_[1],asin(view_matrix_(0,0)),  cvImg);
//		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
//												robotdata_->x_pos_, view_matrix_eye_[0],
//												robotdata_->y_pos_, view_matrix_eye_[1],
//												robotdata_->psi_, asin(view_matrix_(0,0)), cvImg);
//		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
//														robotdata_->x_pos_, view_matrix_eye_[0], localisation_->particles.at(0).xPos,
//														robotdata_->y_pos_, view_matrix_eye_[1], localisation_->particles.at(0).yPos,
//														robotdata_->psi_, asin(view_matrix_(0,0)), localisation_->particles.at(0).psi, cvImg);
		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
												robotdata_->x_pos_, view_matrix_eye_[0], localisation_->particles.at(0).xPos,
												robotdata_->y_pos_, view_matrix_eye_[1], localisation_->particles.at(0).yPos,
												robotdata_->psi_, view_matrix_(0,0), localisation_->particles.at(0).psi, cvImg);
		// observe for particle filter is done here.
		Observe();

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
		//dataWriter->writeData(robotData->incrementeLeft, robotData->incrementeRight, robotData->posX,robotData->posY,robotData->psi);
//		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
//										view_matrix_eye_[0],
//										view_matrix_eye_[1],
//										asin(view_matrix_(0,0)));
//		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
//										robotdata_->x_pos_, view_matrix_eye_[0],
//										robotdata_->y_pos_, view_matrix_eye_[1],
//										robotdata_->psi_, asin(view_matrix_(0,0)));
		data_to_file_writer_.WriteData(robotdata_->incremente_left_, robotdata_->incremente_right_,
														robotdata_->x_pos_, view_matrix_eye_[0], localisation_->particles.at(0).xPos,
														robotdata_->y_pos_, view_matrix_eye_[1], localisation_->particles.at(0).yPos,
														robotdata_->psi_, view_matrix_(0,0), localisation_->particles.at(0).psi);
		old_increments_left_ = robotdata_->incremente_left_;
		old_increments_right_ = robotdata_->incremente_right_;
	}

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
			robotdata_->speed_ = sixaxes_->axis(AXIS_LY)*-0.1;
			robotdata_->psi_speed_ = sixaxes_->axis(AXIS_RX)*-0.04;
		}
		else{
			robotdata_->speed_ = 0.0;
			robotdata_->psi_speed_ = 0.0;
		}
	}
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
	osg::Image* leinwandFace01 = osgDB::readImageFile("/home/josef/workspace/test/wall1.jpg");
	osg::Image* leinwandFace02 = osgDB::readImageFile("/home/josef/workspace/test/wall2.jpg");
	osg::Image* leinwandFace03 = osgDB::readImageFile("/home/josef/workspace/test/wall3.jpg");
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

	default:
		break;
	}
}

void Simulation::Dynamic() {
}



