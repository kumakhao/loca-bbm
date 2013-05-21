/*
 * locaUtil.h
 *
 *  Created on: Feb 4, 2013
 *      Author: josef
 */

#ifndef LOCAUTIL_H_
#define LOCAUTIL_H_
#include <opencv.hpp>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osgViewer/Viewer>

/**
 * A utility class for different functions used on multiple occasions.
 */
class locaUtil
{
public:
	class getWorldCoordOfNodeVisitor : public osg::NodeVisitor
	{
	public:
	   getWorldCoordOfNodeVisitor():
	      osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
	      {
	         wcMatrix= new osg::Matrixd();
	      }
	      virtual void apply(osg::Node &node)
	      {
	         if (!done)
	         {
	            if ( 0 == node.getNumParents() ) // no parents
	            {
	               wcMatrix->set( osg::computeLocalToWorld(this->getNodePath()) );
	               done = true;
	            }
	            traverse(node);
	         }
	      }
	      osg::Matrixd* giveUpDaMat()
	      {
	         return wcMatrix;
	      }
	private:
	   bool done;
	   osg::Matrix* wcMatrix;
	};
	static osg::Matrixd* getWorldCoords( osg::Node* node);


	static const double squareSize = 0.125;		/**< Size of the squares on the lightwall in m */
	static const double squareHeight = 2.9375;	/**< Height of the top lightwall square center above ground. */


	static double angleNormalisation(double in);
	static double randomUniform();
	static double randomGaussian();
	static cv::Mat getBit3Dlocations_1();
	static cv::Mat getBit3Dlocations_2();
	static cv::Mat getBit3Dlocations_3();
	static cv::Mat getBit3Dlocations_all();
	static unsigned char* getPatternAlternate();
	static unsigned char* getPatternCode93();
	static osg::Geode* pyramid();
	static cv::Mat makeWall(unsigned char *pattern, int wallNr);

	private:
	static unsigned char* code93(unsigned char *arr, int a0);
};


#endif /* LOCAUTIL_H_ */
