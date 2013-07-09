/*
 * ScreenShotCallback.h
 *
 *  Created on: Jan 14, 2013
 *      Author: josef
 */

#ifndef SCREENSHOTCALLBACK_H_
#define SCREENSHOTCALLBACK_H_

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/RenderInfo>


class ScreenShotCallback : public osg::Camera::DrawCallback
{
    class ScreenShotMaker
    {
      public:
        ScreenShotMaker() :
            old_w_(-1),
            old_h_(-1),
            do_capture_img_(false)
        {
        }
        void queueShot() { do_capture_img_=true; }
        bool isPicTaken(){return !do_capture_img_;}
        osg::ref_ptr<osg::Image> getImage(){ return osg_image_;};
        void readPixels(osg::RenderInfo& ri)
        {
            if (!do_capture_img_) return;

            glReadBuffer(GL_BACK);
            osg::GraphicsContext* gc = ri.getState()->getGraphicsContext();
            if (!gc->getTraits()) return;
            int w = gc->getTraits()->width;
            int h = gc->getTraits()->height;
            if ((old_w_ != w) || (old_h_ != h)) {
                osg_image_ = new osg::Image();
                osg_image_->allocateImage(w, h, 1,
                                          GL_BGR,  GL_UNSIGNED_BYTE);
                old_w_ = w;
                old_h_ = h;
            }

            osg_image_->readPixels(0, 0, w, h,
                                   GL_BGR, GL_UNSIGNED_BYTE);

            do_capture_img_ = false;
        }
      private:
        osg::ref_ptr<osg::Image> osg_image_;
        int old_w_;
        int old_h_;
        bool do_capture_img_;
    };
  public:
    ScreenShotCallback()
    {
        maker_ = new ScreenShotMaker();
    }
    ~ScreenShotCallback()
    {
        delete maker_;
    };
    void queueShot() {
        maker_->queueShot();
    }
    bool isPicTaken()
    {
    	return maker_->isPicTaken();
    }
    osg::ref_ptr<osg::Image> getImage(){
    	return maker_->getImage();
    }
    virtual void operator()(osg::RenderInfo& ri) const
    {
        maker_->readPixels(ri);
    }
  private:
    ScreenShotMaker* maker_;
};


#endif /* SCREENSHOTCALLBACK_H_ */
