#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui/screen_screen/screenPresenter.hpp>
#include <gui_generated/screen_screen/screenViewBase.hpp>

class screenView : public screenViewBase {
   public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

   protected:
};

#endif  // SCREENVIEW_HPP
