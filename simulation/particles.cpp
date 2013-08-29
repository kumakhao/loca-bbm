/*
 * particles.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: josef
 */


#include <osgDB/ReadFile>
#include <osg/Group>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osgSim/DOFTransform>
#include <osgGA/GUIEventHandler>
#include <osg/ref_ptr>

#include <stdlib.h>
#include <math.h>
#include <vector>

#include "particles.h"
#include "../localisation/localisation.h"

Particles::ParticleDataType::ParticleDataType(osg::Node*n, double x_pos, double y_pos, double orientation)
{
	this->particleXform_ =
			 dynamic_cast <osg::PositionAttitudeTransform*> (n);
	this->x_pos_ = x_pos;
	this->y_pos_ = y_pos;
	this->orientation_ = orientation;
}

void Particles::ParticleDataType::Update()
{
	particleXform_->setPosition(osg::Vec3(x_pos_,y_pos_,z_pos_));
	particleXform_->setAttitude(osg::Quat(orientation_, osg::Vec3d(0.0, 0.0, 1.0)));
}

void Particles::ParticleDataType::setPosition(double x, double y, double psi, double z)
{
	this->x_pos_ = x;
	this->y_pos_ = y;
	this->z_pos_ = z;
	this->orientation_ = psi;
}

void Particles::Update(std::vector<localisation::Particle> particle_data)
{
	int nr_of_particles;
	bool initilisationDone = false;
	if(particle_data.size() > 0)
		initilisationDone = particle_data.at(0).loca->initilisation_done_;
	if(particles_group_.size()>=particle_data.size())
		nr_of_particles = particle_data.size();
	else
		nr_of_particles = particles_group_.size();
	for(int i=0;i<nr_of_particles;i+=1)
	{
		if(i>=nr_of_particles)
			break;
		//pointer
		ParticleDataType* particleData =
				dynamic_cast<ParticleDataType*> (particles_group_.at(i)->getUserData());
		if(initilisationDone)
			particleData->setPosition(particle_data.at(i).xPos, particle_data.at(i).yPos, particle_data.at(i).psi);
		else
			particleData->setPosition(0.0, 0.0, 0.0, -1.0);

	}
}

void Particles::Populate(int size)
{
	particles_group_.clear();
	for(int i=0;i<(size);i++)
	{
		particles_group_.push_back(MakeParticle());
	}
}



osg::Group* Particles::MakeParticle()
{
	osg::PositionAttitudeTransform* particleXform = new osg::PositionAttitudeTransform();
	osg::Geode* particle = new osg::Geode();
	osg::Geometry* particle_geo = new osg::Geometry();
	ParticleDataType* particle_data = new ParticleDataType(particleXform,6,6,0);
	particle->addDrawable(particle_geo);

	// Specify the vertices:
	osg::Vec3Array* particle_vertices = new osg::Vec3Array;
	particle_vertices->push_back( osg::Vec3(0.1, 0, 0) ); // front left
	particle_vertices->push_back( osg::Vec3(-0.1, -0.05, 0) ); // front right
	particle_vertices->push_back( osg::Vec3(-0.1, 0.05, 0) ); // top right
	//particleVertices->push_back( osg::Vec3(-0.1, 0.1, 0) ); // top left

	particle_geo->setVertexArray( particle_vertices );

	osg::DrawElementsUInt* quadrat =
			new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	//quadrat->push_back(3);
	quadrat->push_back(2);
	quadrat->push_back(1);
	quadrat->push_back(0);
	particle_geo->addPrimitiveSet(quadrat);

	osg::Vec2Array* texcoords = new osg::Vec2Array(5);
	(*texcoords)[0].set(0.0f,0.0f); // tex coord for vertex 0
	(*texcoords)[1].set(1.0f,0.0f); // tex coord for vertex 1
	(*texcoords)[2].set(1.0f,1.0f); // ""
	//(*texcoords)[3].set(0.0f,1.0f); // ""
	particle_geo->setTexCoordArray(0,texcoords);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
	//colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );

	particle_geo->setColorArray(colors);
	particle_geo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	particleXform->setPosition(osg::Vec3(6,6,2));
	particleXform->addChild(particle);
	particleXform->setUserData(particle_data);
	particleXform->setAttitude(osg::Quat(0, osg::Vec3d(0.0, 0.0, 1.0)));
	particleXform->setUpdateCallback(new ParticleNodeCallback());
	return (osg::Group*) particleXform;
}
