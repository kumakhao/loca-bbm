/*
 * particles.h
 *
 *  Created on: Jan 21, 2013
 *      Author: josef
 */

#ifndef PARTICLES_H_
#define PARTICLES_H_

#include <osg/PositionAttitudeTransform>
#include "../localisation/localisation.h"

class Particles
{
public:

	class ParticleDataType : public osg::Referenced
	{
	public:
		ParticleDataType(osg::Node*n, double x_pos, double y_pos, double orientation);
		void Update();
		void setPosition(double x, double y, double psi, double z=0.01);
	protected:
		osg::PositionAttitudeTransform* particleXform_;
		double x_pos_, y_pos_, z_pos_, orientation_;
	};

	class ParticleNodeCallback : public osg::NodeCallback
	{
	public:
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
		{
			osg::ref_ptr<ParticleDataType> particleData =
					dynamic_cast<ParticleDataType*> (node->getUserData());
			if(particleData)
			{
				particleData->Update();
			}
			traverse(node, nv);
		}
	};

	std::vector<osg::Group*> particles_group_;
	void Update(std::vector<localisation::Particle> particle_data);
	void Populate(int size);
	void AddToThis(osg::Group* root) {
		for (unsigned int i = 0; i < particles_group_.size(); i++) {
			root->addChild(particles_group_.at(i));
		}
	}
	osg::Group* MakeParticle();
};


#endif /* PARTICLES_H_ */
