//
// "$Id: CycleButton.h 4910 2006-04-06 19:26:22Z fabien $"
//
// Popup list of items that the user can choose one of. Based on Motif
// but modern equivalent is the OS/X popup choices.
//
// This is not a "combo box". You should try this, but if you insist
// on that use the InputBrowser widget.
//
// Copyright 2004 by Bill Spitzak and others.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA.
//
// Please report all bugs and problems to "fltk-bugs@fltk.org".
//

#ifndef fltk_CycleButton_h
#define fltk_CycleButton_h

#include "Menu.h"

namespace fltk {

class FL_API CycleButton : public Menu {
public:
  int handle(int);
  CycleButton(int,int,int,int,const char * = 0);
  static NamedStyle* default_style;
protected:
  void draw();
};

}

#endif

//
// End of "$Id: CycleButton.h 4910 2006-04-06 19:26:22Z fabien $".
//
