/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/10/4

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */
#ifndef SDL_AVATAR_H
#define SDL_AVATAR_H

//#define DEBUG // to activate all the printf(), also used in sdl_utils

// utils
#include "vision_utils/all_files_in_dir.h"
#include "vision_utils/all_subfolders.h"
#include "vision_utils/clamp.h"
#include "vision_utils/rand_gaussian.h"
#include "vision_utils/replace_find_tags.h"
#include "vision_utils/sdl_utils.h"
#include "vision_utils/XmlDocument.h"
// C
#include <dirent.h>

//! \return false if problem
inline bool imread_all_files_in_dir(SDL_Renderer* renderer,
                                    const std::string & folder,
                                    std::vector<Texture> & ans,
                                    const std::string pattern = "") {
  DEBUG_PRINT("imread_all_files_in_dir('%s', pattern:'%s')\n",
              folder.c_str(), pattern.c_str());
  std::vector<std::string> filenames;
  vision_utils::all_files_in_dir(folder, filenames, pattern);
  unsigned int nfiles = filenames.size();
  // now read images
  ans.resize(nfiles);
  for (unsigned int i = 0; i < nfiles; ++i) {
    //DEBUG_PRINT("filename:'%s'\n", filenames[i].c_str());
    if (!ans[i].from_file(renderer, filenames[i]))
      return false;
  } // end loop i
  return true;
} // end imread_all_files_in_dir()

inline bool replace_colors(SDL_Surface * in, SDL_Surface * out,
                           SDL_Color find, SDL_Color replacewith) {
  unsigned int w = in->w, h = in->h;
  if(!in || !w || !h)
    return false;
  Uint32 f = color2int(find), r = color2int(replacewith);
  SDL_FreeSurface(out);
  SDL_BlitSurface(in, NULL, out, NULL);
  //  *out = SDL_CreateRGBSurface(in->flags, w, h, in->format->BitsPerPixel,
  //                              in->format->Rmask, in->format->Gmask,
  //                              in->format->Bmask, in->format->Amask);
  for(unsigned int y = 0; y < h; y++) {
    for(unsigned int x = 0; x < w; x++) {
      if (getpixel(in, x, y) == f)
        putpixel(out, x, y, r);
    } // end for x;
  } // end for y
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class AvatarRenderable {
public:
  enum Type {
    TYPE_UNDEFINED = 0,
    TYPE_EYE = 1,
    TYPE_BINARY_LED = 2,
    TYPE_COLOR_LED = 3
  };

  AvatarRenderable() : _type(TYPE_UNDEFINED) {}

  virtual ~AvatarRenderable() {}

  virtual bool render_center(SDL_Renderer* renderer,
                             const Point2i & center_position,
                             bool flip) = 0;

  //////////////////////////////////////////////////////////////////////////////

  std::string get_name()  const { return _name; }
  void set_name(const std::string & n) { _name = n; }
  Type get_type() const { return _type; }
  std::string _name;
  std::vector<Point2i> _center_pos;
  std::vector<bool> _flips;
protected:
  Type _type;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Eye : public AvatarRenderable {
public:
  typedef std::string StateName;
  enum Substate { OPEN = 0, BEGIN = 2, BLINK = 3, END = 4 };
  class StateData {
  public:
    StateData() : _blink_period(3), _next_blink_time(3), _eyelid_duration(.06) {}
    StateName _name;
    double _blink_period, _next_blink_time, _eyelid_duration;
    std::vector<Texture> _eyelids_blink,  _eyelids_begin,  _eyelids_open;
    unsigned int           _neyelids_blink,  _neyelids_begin,  _neyelids_open;
  }; // end class StateData

  Eye() {
    _type = TYPE_EYE;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool from_imgs(SDL_Renderer* renderer,
                 const std::string & eyes_folder = "") {
    DEBUG_PRINT("Eye::from_imgs('%s')\n", eyes_folder.c_str());
    _states_data.clear();
    if (eyes_folder.empty())
      return from_default_imgs(renderer);
    if (!boost::filesystem::exists(eyes_folder)
        || !boost::filesystem::is_directory(eyes_folder)) {
      printf("Directory '%s' doesn't exist!\n", eyes_folder.c_str());
      return false;
    }
    // load background RGB
    if (!_bg.from_file(renderer, eyes_folder + "/bg.png"))
      return false;
    // load iris RGBA
    if (!_iris.from_file(renderer, eyes_folder + "/iris.png"))
      return false;
    _iris_transmax_x = (_bg.get_width() - _iris.get_width()) / 2;
    _iris_transmax_y = (_bg.get_height() - _iris.get_height()) / 2;
    // load all states
    std::vector<std::string> states;
    vision_utils::all_subfolders(eyes_folder, states);
    for (unsigned int i = 0; i < states.size(); ++i) {
      if (!load_state(renderer, states[i], eyes_folder + "/" + states[i]))
        return false;
    }
    if (_states_data.empty()) {
      printf("Could not load any state!\n");
      return false;
    }

    // safe default values
    StateName first_state = _states_data.begin()->second._name;
    if (!set_state_notransition("normal")
        && !set_state_notransition(first_state)) {
      printf("Could not set state 'normal' nor first state '%s'!\n",
             first_state.c_str());
      return false;
    }
    move_iris(0, 0);
    return true;
  } // end from_imgs();

  //////////////////////////////////////////////////////////////////////////////

  bool from_default_imgs(SDL_Renderer* renderer) {
    DEBUG_PRINT("Eye::from_default_imgs(renderer)\n");
    _states_data.clear();
    std::string path = ros::package::getPath("rosavatar") + "/data/mini_eyes";
    return from_imgs(renderer, path);
  } // end from_default_imgs(renderer)

  //////////////////////////////////////////////////////////////////////////////

  bool set_state(const StateName & state) {
    DEBUG_PRINT("Eye::set_state('%s')\n", state.c_str());
    // check state exists
    if (_states_data.count(state) == 0) {
      printf("Eye::set_state('%s'): set does not exist\n", state.c_str());
      return false;
    }
    if (_curr_state.empty()) // there was no previous state
      return set_state_notransition(state);
    DEBUG_PRINT("Queued state '%s'\n", state.c_str());
    _next_state = state; // we will need to switch to SubState END
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! irisx, irisy in [-1, 1]
  inline void move_iris(double irisx, double irisy) {
    DEBUG_PRINT("Eye::move_iris(%g,%g)\n", irisx, irisy);
    _irisx = vision_utils::clamp(irisx, -1., 1.);
    _irisy = vision_utils::clamp(irisy, -1., 1.);
    _iris_trans.x = _irisx * _iris_transmax_x;
    _iris_trans.y = _irisy * _iris_transmax_y;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool render_center(SDL_Renderer* renderer,
                     const Point2i & center_position,
                     bool flip) {
    DEBUG_PRINT("Eye::render_center()\n");
    if (_curr_state.empty()) {
      printf("Eye::render_center(): _curr_state undefined!\n");
      return false;
    }
    // determine what eyelid needs to be drawn
    double time = _curr_substate_timer.getTimeSeconds();
    unsigned int next_eyelid_idx = 1. * time / _curr_statedata->_eyelid_duration;
    Texture* next_eyelid = NULL;

    if (_curr_substate == BEGIN) {
      // check for transitions to OPEN
      if (next_eyelid_idx >= _curr_statedata->_neyelids_begin) {
        if (_next_state != _curr_state) { // state queued -> end the current state
          DEBUG_PRINT("'%s':BEGIN -> END\n", _curr_state.c_str());
          _curr_substate = END;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_begin.back());
        } else { // no state queued
          DEBUG_PRINT("'%s':BEGIN -> OPEN\n", _curr_state.c_str());
          _curr_substate = OPEN;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_open.front());
        }
      }
      else { // default BEGIN animation
        DEBUG_PRINT("'%s':default BEGIN\n", _curr_state.c_str());
        next_eyelid = &(_curr_statedata->_eyelids_begin.at(next_eyelid_idx));
      }
    } // end BEGIN

    else if (_curr_substate == BLINK) {
      // check for transitions to OPEN
      if (next_eyelid_idx >= _curr_statedata->_neyelids_blink) {
        if (_next_state != _curr_state) { // state queued -> end the current state
          DEBUG_PRINT("'%s':BLINK -> END\n", _curr_state.c_str());
          _curr_substate = END;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_begin.back());
        } else {
          DEBUG_PRINT("'%s':BLINK -> OPEN\n", _curr_state.c_str());
          _curr_substate = OPEN;
          _curr_substate_timer.reset();
          next_eyelid = &(_curr_statedata->_eyelids_open.front());
          // gaussian probability for next blink
          double blink = .3 * vision_utils::rand_gaussian() + _curr_statedata->_blink_period;
          blink = std::max(blink, 1.);
          _curr_statedata->_next_blink_time = blink;
        }
      }
      else { // default OPEN BLINK eyelid
        DEBUG_PRINT("'%s':default OPEN BLINK\n", _curr_state.c_str());
        next_eyelid = &(_curr_statedata->_eyelids_blink.at(next_eyelid_idx));
      }
    } // end BLINK

    else if (_curr_substate == END) {
      if (next_eyelid_idx >= _curr_statedata->_neyelids_begin) {
        DEBUG_PRINT("'%s':END -> BEGIN '%s'\n",
                    _curr_state.c_str(), _next_state.c_str());
        if (!set_state_notransition(_next_state))
          return false;
        next_eyelid = &(_curr_statedata->_eyelids_begin.front());
      }
      else { // default END animation = reverse BEGIN
        DEBUG_PRINT("'%s':default END\n", _curr_state.c_str());
        unsigned int idx = _curr_statedata->_neyelids_begin - 1 - next_eyelid_idx;
        next_eyelid = &(_curr_statedata->_eyelids_begin.at(idx));
      }
    } // end END

    else /*if (_curr_substate == OPEN)*/ {
      // check for transitions to blink
      if (_next_state != _curr_state) { // state queued -> end the current state
        DEBUG_PRINT("'%s':OPEN -> END\n", _curr_state.c_str());
        _curr_substate = END;
        _curr_substate_timer.reset();
        next_eyelid = &(_curr_statedata->_eyelids_begin.back());
      } else if (time > _curr_statedata->_next_blink_time) { // timeout for blink
        DEBUG_PRINT("'%s':OPEN -> BLINK\n", _curr_state.c_str());
        _curr_substate = BLINK;
        _curr_substate_timer.reset();
        next_eyelid = &(_curr_statedata->_eyelids_blink.front());
      }
      else { // default OPEN eyelid
        DEBUG_PRINT("'%s':default OPEN\n", _curr_state.c_str());
        next_eyelid = &(_curr_statedata->_eyelids_open.front());
      }
    } // end OPEN

    if (next_eyelid == NULL) {
      printf("next_eyelid = NULL, something went wrong...\n");
      return false;
    }
    // force render if iris has moved a lot
    DEBUG_PRINT("Eye::renderer(): rendering bg...\n");
    _bg.render_center(renderer, center_position);
    DEBUG_PRINT("Eye::renderer(): rendering iris...\n");
    _iris.render_center(renderer, center_position + _iris_trans);
    DEBUG_PRINT("Eye::renderer(): rendering eyelid...\n");
    SDL_RendererFlip sdlflip = (flip ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);
    next_eyelid->render_center(renderer, center_position, 1, NULL, 0,
                               Point2d(-1,-1), sdlflip);
    _curr_eyelid = next_eyelid;
    return true;
  } // end render_center()

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////

  bool set_state_notransition(const StateName & state) {
    DEBUG_PRINT("Eye::set_state_notransition('%s')\n", state.c_str());
    std::map<StateName, StateData>::iterator it = _states_data.find(state);
    if (it == _states_data.end()) {
      printf("Eye::set_state_notransition('%s'): could not set state\n", state.c_str());
      return false;
    }
    _next_state = _curr_state = state;
    _curr_statedata = &(it->second);
    _curr_substate = BEGIN;
    _curr_substate_timer.reset();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! load the eyelids of a given state
  inline bool load_state(SDL_Renderer* renderer,
                         const StateName & state,
                         const std::string & folder) {
    DEBUG_PRINT("Eye::load_state('%s')\n", state.c_str());
    bool ok = (_states_data.insert(std::make_pair(state, StateData()))).second;
    if (!ok) {
      printf("Eye::load_state('%s'): could not insert the state!\n", state.c_str());
      return false;
    }
    StateData* data = &(_states_data[state]);
    data->_name = state;
    if (!imread_all_files_in_dir(renderer, folder, data->_eyelids_begin, "begin_")
        || !imread_all_files_in_dir(renderer, folder, data->_eyelids_blink, "blink_")
        || !imread_all_files_in_dir(renderer, folder, data->_eyelids_open, "open_")) {
      printf("Eye::load_state('%s'): could not load one of the states!\n", state.c_str());
      return false;
    }
    data->_neyelids_begin = data->_eyelids_begin.size();
    data->_neyelids_blink = data->_eyelids_blink.size();
    data->_neyelids_open = data->_eyelids_open.size();
    return true;
  } // end load_state()

  //////////////////////////////////////////////////////////////////////////////

  double _irisx, _irisy;
  int _iris_transmax_x, _iris_transmax_y;
  Point2i _iris_trans;
  Texture _iris, _bg;
  std::map<StateName, StateData> _states_data;
  Texture* _curr_eyelid; //!< pointer to the current eyelid frame
  StateData* _curr_statedata;  //!< pointer to the current state data
  Substate _curr_substate;
  vision_utils::Timer _curr_substate_timer;
  StateName _curr_state, _next_state;
  unsigned int _curr_substate_idx;
}; // end class Eye

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


class BinaryLed : public AvatarRenderable {
public:
  enum State { OFF = 0, ON = 1 };

  BinaryLed() : _state(OFF), _auto_mode(false) {
    _type = TYPE_BINARY_LED;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool from_imgs(SDL_Renderer* renderer,
                 const std::string & led_prefix = "") {
    DEBUG_PRINT("BinaryLed::from_imgs('%s')\n", led_prefix.c_str());
    if (led_prefix.empty())
      return from_default_imgs(renderer);
    std::ostringstream on_file, off_file;
    on_file << led_prefix << "_on.png";
    off_file << led_prefix << "_off.png";
    return (!_on_img.from_file(renderer, on_file.str())
            || _off_img.from_file(renderer, off_file.str()));
  } // end from_imgs();

  //////////////////////////////////////////////////////////////////////////////

  inline void set_state(BinaryLed::State state) { _state = state; }

  //////////////////////////////////////////////////////////////////////////////

  void set_auto_mode(const double & auto_mode_threshold) {
    _auto_mode = true;
    _auto_mode_threshold = auto_mode_threshold;
  }
  inline void unset_auto_mode() { _auto_mode = false; }
  inline bool get_auto_mode() const { return _auto_mode; }
  inline double get_auto_mode_threshold() const { return _auto_mode_threshold; }
  inline void set_auto_mode_value(double auto_mode_value) {
    // apply threshold if needed
    if (_auto_mode)
      _state = (auto_mode_value > _auto_mode_threshold ? ON : OFF);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool render_center(SDL_Renderer* renderer,
                     const Point2i & center_position,
                     bool flip) {
    DEBUG_PRINT("BinaryLed::render_center()\n");
    Texture* to_copy = &(_state == OFF ? _off_img : _on_img);
    SDL_RendererFlip sdlflip = (flip ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);
    to_copy->render_center(renderer, center_position, 1, NULL, 0,
                           Point2d(-1,-1), sdlflip);
    return true;
  } // end render_center();

  //////////////////////////////////////////////////////////////////////////////

protected:
  bool from_default_imgs(SDL_Renderer* renderer) {
    std::string path = ros::package::getPath("rosavatar") + "/data/leds/blue";
    return from_imgs(renderer, path);
  }

  State _state;
  bool _auto_mode;
  Texture _off_img, _on_img;
  double _auto_mode_threshold; //! threshold for on/off
}; // end class Led

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class ColorLed : public AvatarRenderable {
public:
  typedef SDL_Color Color;

  ColorLed() {
    _type = TYPE_COLOR_LED;
  }

  bool from_imgs(SDL_Renderer* renderer,
                 const std::string & led_prefix = "") {
    DEBUG_PRINT("ColorLed::from_imgs('%s')\n", led_prefix.c_str());
    if (led_prefix.empty())
      return from_default_imgs(renderer);
    std::ostringstream bg_file, top_file;
    bg_file << led_prefix << "_bg.png";
    top_file << led_prefix << "_top.png";
    return (!_bg_img.from_file(renderer, bg_file.str())
            || _top_img.from_file(renderer, top_file.str()));
  } // end from_imgs();

  //////////////////////////////////////////////////////////////////////////////

  inline bool set_color(SDL_Renderer* renderer, Color c) {
    SDL_Surface* tmp;
    _color = c;
    if (!replace_colors(_bg_img.get_sdlsurface_raw(), tmp, _tofind_color, _color)
        || !_bg_tinted_img.from_surface(tmp, renderer))
      return false;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool render_center(SDL_Renderer* renderer,
                     const Point2i & center_position,
                     bool flip) {
    SDL_RendererFlip sdlflip = (flip ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);
    _bg_img.render_center(renderer, center_position, 1, NULL, 0,
                          Point2d(-1,-1), sdlflip);
    _top_img.render_center(renderer, center_position, 1, NULL, 0,
                           Point2d(-1,-1), sdlflip);
    return true;
  } // end render_center();

  //////////////////////////////////////////////////////////////////////////////

protected:
  bool from_default_imgs(SDL_Renderer* renderer) {
    std::string path = ros::package::getPath("rosavatar") + "/data/leds/binary";
    return from_imgs(renderer, path);
  }

  Color _tofind_color, _color;
  Texture _top_img, _bg_img, _bg_tinted_img;
}; // end class Led

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class SDLAvatar {
public:
  typedef vision_utils::XmlDocument::Node Node;

  SDLAvatar()  : _init_done(false), _renderer(NULL), _window(NULL) {
  }

  bool init(int winw = 640, int winh = 480,
            Uint32 win_flags = 0) {
    DEBUG_PRINT("init(%ix%i)\n", winw, winh);
    if (_init_done)
      return true;
    _renderer = NULL;
    _window = NULL;
    _bg_color = SDL_Color_ctor(0, 0, 0);
    // init SDL
    SDL_Init( SDL_INIT_EVERYTHING );
    //SDL_CreateWindowAndRenderer(winw, winh, 0, &window, &renderer);
    // Set size of renderer to the same as window
    //SDL_RenderSetLogicalSize( renderer, winw, winh );

    // create window
    SDL_Rect windowRect = { 10, 10, winw, winh};
    _window = SDL_CreateWindow( "Avatar", windowRect.x, windowRect.y, winw, winh,
                                win_flags);
    if ( _window == NULL ) {
      printf("Failed to create window:'%s'\n", SDL_GetError());
      return false;
    }
    // create renderer
    _renderer = SDL_CreateRenderer( _window, -1, 0 );
    if ( _renderer == NULL ) {
      printf("Failed to create renderer:'%s'\n", SDL_GetError());
      return false;
    }
    // Set size of renderer to the same as window
    SDL_RenderSetLogicalSize( _renderer, winw, winh );

    //Initialize PNG loading
    int imgFlags = IMG_INIT_PNG;
    if( !( IMG_Init( imgFlags ) & imgFlags ) ) {
      printf( "SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError() );
      return false;
    }
    _init_done = true;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  ~SDLAvatar() {
    // clear components
    for (unsigned int i = 0; i < _renderables.size(); ++i)
      delete _renderables[i];
    _renderables.clear();
    if (_renderer)
      SDL_DestroyRenderer(_renderer);
    if (_window)
      SDL_DestroyWindow(_window);
    IMG_Quit();
    SDL_Quit();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool renderable_from_xml_node_list(vision_utils::XmlDocument & doc,
                                     std::vector<Node*> nodes,
                                     AvatarRenderable::Type type) {
    std::vector<Node*> center_nodes;
    for (unsigned int i = 0; i < nodes.size(); ++i) {
      Node* rnode = nodes[i];
      std::string folder = doc.get_node_attribute(rnode, "folder");
      std::string folder_clean = vision_utils::replace_find_tags(folder);
      DEBUG_PRINT("Renderable folder:'%s'\n", folder.c_str());
      AvatarRenderable* renderable;
      std::string safe_name;
      if (type == AvatarRenderable::TYPE_EYE) {
        safe_name = std::string("eye") + vision_utils::cast_to_string(1+neyes());
        renderable = new Eye();
        Eye* eye = (Eye*) renderable;
        if (!eye->from_imgs(_renderer, folder_clean))
          return false;
      }
      else if (type == AvatarRenderable::TYPE_BINARY_LED) {
        safe_name = std::string("binary_led") + vision_utils::cast_to_string(1+nbinary_leds());
        renderable = new BinaryLed();
        BinaryLed* led = (BinaryLed*) renderable;
        if (!led->from_imgs(_renderer, folder_clean))
          return false;
        bool has_auto_mode = doc.has_node_attribute(rnode, "auto_mode_threshold");
        double auto_mode_threshold = doc.get_node_attribute(rnode, "auto_mode_threshold", -1.0);
        if (has_auto_mode)
          led->set_auto_mode(auto_mode_threshold);

      }
      else if (type == AvatarRenderable::TYPE_COLOR_LED) {
        safe_name = std::string("color_led") + vision_utils::cast_to_string(1+ncolor_leds());
        renderable = new ColorLed();
        ColorLed* led = (ColorLed*) renderable;
        if (!led->from_imgs(_renderer, folder_clean))
          return false;
      }
      else {
        ROS_WARN("Unknown type: %i", type);
        return false;
      }
      // parse centers and flips
      doc.get_all_nodes_at_direction(rnode, "center", center_nodes);
      for (unsigned int j = 0; j < center_nodes.size(); ++j) {
        Node* center_node = center_nodes[j];
        int x = doc.get_node_attribute(center_node, "x", 0);
        int y = doc.get_node_attribute(center_node, "y", 0);
        bool flip = doc.get_node_attribute(center_node, "flip", false);
        renderable->_center_pos.push_back(Point2i(x, y));
        renderable->_flips.push_back(flip);
      } // end for j
      // load eye imgs if needed
      std::string xml_name = doc.get_node_attribute(rnode, "name");
      renderable->set_name(xml_name.empty() ? safe_name : xml_name);

      _renderables.push_back(renderable);
    } // end for i
    return true; // success
  }

  bool from_xml_file(const std::string & filename,
                     Uint32 win_flags = 0) {
    vision_utils::XmlDocument doc;
    if (!doc.load_from_file(vision_utils::replace_find_tags(filename)))
      return false;
    // get background property
    Node* bg = doc.get_node_at_direction(doc.root(), "background");
    if (!bg) {
      printf("Missing background node!\n");
      return false;
    }
    int width  = doc.get_node_attribute(bg, "width", 640);
    int height = doc.get_node_attribute(bg, "height", 480);
    if (!init(width, height, win_flags))
      return false;
    std::string bg_file = doc.get_node_attribute(bg, "file");
    if (bg_file.size()) { // use background image file
      if (!_bg.from_file(_renderer, vision_utils::replace_find_tags(bg_file)))
        return false;
    }
    else { // use background color
      int r = doc.get_node_attribute(bg, "r", 0);
      int g = doc.get_node_attribute(bg, "g", 0);
      int b = doc.get_node_attribute(bg, "b", 0);
      _bg_color = SDL_Color_ctor(r, g, b);
    }

    std::vector<Node*> rnodes;
    doc.get_all_nodes_at_direction(doc.root(), "eye", rnodes);
    if (!renderable_from_xml_node_list(doc, rnodes, AvatarRenderable::TYPE_EYE))
      return false;

    doc.get_all_nodes_at_direction(doc.root(), "binary_led", rnodes);
    if (!renderable_from_xml_node_list(doc, rnodes, AvatarRenderable::TYPE_BINARY_LED))
      return false;

    doc.get_all_nodes_at_direction(doc.root(), "color_led", rnodes);
    if (!renderable_from_xml_node_list(doc, rnodes, AvatarRenderable::TYPE_COLOR_LED))
      return false;
    return true; // success
  }

  //////////////////////////////////////////////////////////////////////////////

  bool load_default_avatar(Uint32 win_flags = 0) {
    DEBUG_PRINT("Avatar::load_default_avatar()\n");
    unsigned int w = 640, h = 480;
    if (!init(w, h, win_flags))
      return false;
    // add eyes
    Eye* eye  = new Eye();
    eye->_center_pos.push_back(Point2i(  w/3, h/4));
    eye->_flips.push_back(false);
    eye->_center_pos.push_back(Point2i(2*w/3, h/4));
    eye->_flips.push_back(true);
    if (!eye->from_imgs(_renderer, ""))
      return false;
    // leds
    unsigned int nleds = 5;
    for (unsigned int iled = 1; iled <= nleds; ++iled) {
      double led_ratio = 1. * abs(iled - .5*(nleds+1)) / (.5*(nleds+1));
      BinaryLed* led = new BinaryLed();
      if (!led->from_imgs(_renderer, ""))
        return false;
      led->set_auto_mode(led_ratio);
      led->_center_pos.push_back(Point2i(iled*w/(nleds+1), 3*h/4 - led_ratio * h/4));
      led->_flips.push_back(false);
      _renderables.push_back(led);
    }
    return render();
  } // end load_default_avatar();

  //////////////////////////////////////////////////////////////////////////////

  bool set_eyes_state(const Eye::StateName & state){
    for (unsigned int i = 0; i < neyes(); ++i) {
      if (_renderables[i]->get_type() != AvatarRenderable::TYPE_EYE)
        continue;
      Eye* eye = (Eye*) _renderables[i];
      if (!eye->set_state(state))
        return false;
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! irisx, irisy in [-1, 1]
  inline void move_iris(double irisx, double irisy) {
    for (unsigned int i = 0; i < neyes(); ++i) {
      if (_renderables[i]->get_type() != AvatarRenderable::TYPE_EYE)
        continue;
      Eye* eye = (Eye*) _renderables[i];
      eye->move_iris(irisx, irisy);
    } // end for i
  }

  //////////////////////////////////////////////////////////////////////////////

  bool render() {
    DEBUG_PRINT("Avatar::render_center()\n");
    SDL_RenderClear( _renderer );
    if (_bg.empty()) {  // fill with background color
      SDL_SetRenderDrawColor( _renderer,
                              _bg_color.r, _bg_color.g, _bg_color.b, _bg_color.a);
      int w, h;
      SDL_GetRendererOutputSize(_renderer, &w, &h);
      SDL_Rect rec = SDL_Rect_ctor(0,0,w,h);
      SDL_RenderFillRect(_renderer, &rec); // fill the background
    } else {
      _bg.render(_renderer, Point2i());
    }
    // AvatarRenderables
    for (unsigned int i = 0; i < nrenderables(); ++i) {
      AvatarRenderable* renderable = (_renderables[i]);
      unsigned int npos = renderable->_center_pos.size();
      if (npos != renderable->_flips.size()) {
        ROS_WARN("Avatar::render_center(): insconsisitent data size %i != %li",
                 npos, renderable->_flips.size());
        return false;
      }
      for (unsigned int j = 0; j < npos; ++j) {
        renderable->render_center(_renderer,
                                  renderable->_center_pos[j],
                                  renderable->_flips[j]);
      } // end for j
    } // end for i
    SDL_RenderPresent( _renderer);
    return true;
  } // end render_center()

  //////////////////////////////////////////////////////////////////////////////

  inline SDL_Color    get_bg_color()                     const { return _bg_color; }
  inline Eye *        get_eye(int i)                     const { return (Eye*) _renderables[i]; }
  inline const AvatarRenderable*  get_renderable(int i)  const { return _renderables[i]; }
  inline AvatarRenderable::Type   get_rtype(int i)       const { return _renderables[i]->get_type(); }
  inline unsigned int nrenderables()                     const { return _renderables.size(); }
  inline unsigned int nrenderables_rois()                const {
    unsigned int ans = 0;
    for (unsigned int i = 0; i < _renderables.size(); ++i)
      ans += _renderables[i]->_center_pos.size();
    return ans;
  }
  inline unsigned int ntype(AvatarRenderable::Type type) const {
    unsigned int ans = 0;
    for (unsigned int i = 0; i < _renderables.size(); ++i) {
      if (_renderables[i]->get_type() == type)
        ans ++;
    }
    return ans;
  }
  inline unsigned int neyes() const        { return ntype(AvatarRenderable::TYPE_EYE); }
  inline unsigned int nbinary_leds() const { return ntype(AvatarRenderable::TYPE_BINARY_LED); }
  inline unsigned int ncolor_leds() const  { return ntype(AvatarRenderable::TYPE_COLOR_LED); }


protected:

  //////////////////////////////////////////////////////////////////////////////

  std::vector<AvatarRenderable*> _renderables;
  // SDL stuff
  bool _init_done;
  SDL_Renderer* _renderer;
  SDL_Window* _window;
  SDL_Color _bg_color;
  Texture _bg;
}; // end class SDLAvatar

#endif // SDL_AVATAR_H
