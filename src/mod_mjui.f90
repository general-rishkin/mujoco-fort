module mod_mjui
  use iso_c_binding
  use mod_mjrender
  implicit none
  
  integer(c_int), parameter :: mjMAXUISECT     = 10      !! maximum number of sections
  integer(c_int), parameter :: mjMAXUIITEM     = 80      !! maximum number of items per section
  integer(c_int), parameter :: mjMAXUITEXT     = 300     !! maximum number of chars in edittext and other
  integer(c_int), parameter :: mjMAXUINAME     = 40      !! maximum number of chars in name
  integer(c_int), parameter :: mjMAXUIMULTI    = 35      !! maximum number of radio/select items in group
  integer(c_int), parameter :: mjMAXUIEDIT     = 7       !! maximum number of elements in edit list
  integer(c_int), parameter :: mjMAXUIRECT     = 25      !! maximum number of rectangles

  integer(c_int), parameter :: mjSEPCLOSED     = 1000    !! closed state of adjustable separator


  !! key codes matching GLFW (user must remap for other frameworks)
  integer(c_int), parameter :: mjKEY_ESCAPE    = 256
  integer(c_int), parameter :: mjKEY_ENTER     = 257
  integer(c_int), parameter :: mjKEY_TAB       = 258
  integer(c_int), parameter :: mjKEY_BACKSPACE = 259
  integer(c_int), parameter :: mjKEY_INSERT    = 260
  integer(c_int), parameter :: mjKEY_DELETE    = 261
  integer(c_int), parameter :: mjKEY_RIGHT     = 262
  integer(c_int), parameter :: mjKEY_LEFT      = 263
  integer(c_int), parameter :: mjKEY_DOWN      = 264
  integer(c_int), parameter :: mjKEY_UP        = 265
  integer(c_int), parameter :: mjKEY_PAGE_UP   = 266
  integer(c_int), parameter :: mjKEY_PAGE_DOWN = 267
  integer(c_int), parameter :: mjKEY_HOME      = 268
  integer(c_int), parameter :: mjKEY_END       = 269
  integer(c_int), parameter :: mjKEY_F1        = 290
  integer(c_int), parameter :: mjKEY_F2        = 291
  integer(c_int), parameter :: mjKEY_F3        = 292
  integer(c_int), parameter :: mjKEY_F4        = 293
  integer(c_int), parameter :: mjKEY_F5        = 294
  integer(c_int), parameter :: mjKEY_F6        = 295
  integer(c_int), parameter :: mjKEY_F7        = 296
  integer(c_int), parameter :: mjKEY_F8        = 297
  integer(c_int), parameter :: mjKEY_F9        = 298
  integer(c_int), parameter :: mjKEY_F10       = 299
  integer(c_int), parameter :: mjKEY_F11       = 300
  integer(c_int), parameter :: mjKEY_F12       = 301

  !---------------------------------- primitive types (mjt) -----------------------------------------

  enum, bind(c) !! mjtButton         !! mouse button
    enumerator :: mjBUTTON_NONE = 0              !! no button
    enumerator :: mjBUTTON_LEFT                  !! left button
    enumerator :: mjBUTTON_RIGHT                 !! right button
    enumerator :: mjBUTTON_MIDDLE                 !! middle button
  end enum !! mjtButton


  enum, bind(c) !! mjtEvent          !! mouse and keyboard event type
    enumerator :: mjEVENT_NONE = 0               !! no event
    enumerator :: mjEVENT_MOVE                   !! mouse move
    enumerator :: mjEVENT_PRESS                  !! mouse button press
    enumerator :: mjEVENT_RELEASE                !! mouse button release
    enumerator :: mjEVENT_SCROLL                 !! scroll
    enumerator :: mjEVENT_KEY                    !! key press
    enumerator :: mjEVENT_RESIZE                  !! resize
  end enum !! mjtEvent


  enum, bind(c) !! mjtItem           !! UI item type
    enumerator :: mjITEM_END        = -2                !! end of definition list (not an item)
    enumerator :: mjITEM_SECTION    = -1            !! section (not an item)
    enumerator :: mjITEM_SEPARATOR  = 0           !! separator
    enumerator :: mjITEM_STATIC                  !! static text
    enumerator :: mjITEM_BUTTON                  !! button

    !! the rest have data pointer
    enumerator :: mjITEM_CHECKINT                !! check box int value
    enumerator :: mjITEM_CHECKBYTE               !! check box mjtByte value
    enumerator :: mjITEM_RADIO                   !! radio group
    enumerator :: mjITEM_RADIOLINE               !! radio group single line
    enumerator :: mjITEM_SELECT                  !! selection box
    enumerator :: mjITEM_SLIDERINT               !! slider int value
    enumerator :: mjITEM_SLIDERNUM               !! slider mjtNum value
    enumerator :: mjITEM_EDITINT                 !! editable array int values
    enumerator :: mjITEM_EDITNUM                 !! editable array mjtNum values
    enumerator :: mjITEM_EDITTXT                 !! editable text

    enumerator :: mjNITEM                         !! number of item types
  end enum !! mjtItem


  !! predicate function: set enable/disable based on item category
  abstract interface
    ! typedef int (*mjfItemEnable)(int category, void* data)
    integer(c_int) function mjfItemEnable(category, data) bind(c)
      import :: c_int, c_ptr
      integer(c_int), value, intent(in) :: category
      type(c_ptr), intent(inout)        :: data
    end function mjfItemEnable
  end interface


  !---------------------------------- mjuiState -----------------------------------------------------

  type, bind(c) :: mjuiState               !! mouse and keyboard state
    !! constants set by user
    integer(c_int)                :: nrect                      !! number of rectangles used
    type(mjrRect)                 :: rect(mjMAXUIRECT)      !! rectangles (index 0: entire window)
    type(c_ptr)                   :: userdata                 !! pointer to user data (for callbacks)

    !! event type
    integer(c_int)                :: type                       !! (type mjtEvent)

    !! mouse buttons
    integer(c_int)                :: left                       !! is left button down
    integer(c_int)                :: right                      !! is right button down
    integer(c_int)                :: middle                     !! is middle button down
    integer(c_int)                :: doubleclick                !! is last press a double click
    integer(c_int)                :: button                     !! which button was pressed (mjtButton)
    real(c_double)                :: buttontime              !! time of last button press

    !! mouse position
    real(c_double)                :: x                       !! x position
    real(c_double)                :: y                       !! y position
    real(c_double)                :: dx                      !! x displacement
    real(c_double)                :: dy                      !! y displacement
    real(c_double)                :: sx                      !! x scroll
    real(c_double)                :: sy                      !! y scroll

    !! keyboard
    integer(c_int)                :: control                    !! is control down
    integer(c_int)                :: shift                      !! is shift down
    integer(c_int)                :: alt                        !! is alt down
    integer(c_int)                :: key                        !! which key was pressed
    real(c_double)                :: keytime                 !! time of last key press

    !! rectangle ownership and dragging
    integer(c_int)                :: mouserect                  !! which rectangle contains mouse
    integer(c_int)                :: dragrect                   !! which rectangle is dragged with mouse
    integer(c_int)                :: dragbutton                 !! which button started drag (mjtButton)
  end type mjuiState


  !---------------------------------- mjuiThemeSpacing ----------------------------------------------

  type, bind(c) :: mjuiThemeSpacing        !! UI visualization theme spacing
    integer(c_int)                :: total                      !! total width
    integer(c_int)                :: scroll                     !! scrollbar width
    integer(c_int)                :: label                      !! label width
    integer(c_int)                :: section                    !! section gap
    integer(c_int)                :: itemside                   !! item side gap
    integer(c_int)                :: itemmid                    !! item middle gap
    integer(c_int)                :: itemver                    !! item vertical gap
    integer(c_int)                :: texthor                    !! text horizontal gap
    integer(c_int)                :: textver                    !! text vertical gap
    integer(c_int)                :: linescroll                 !! number of pixels to scroll
    integer(c_int)                :: samples                    !! number of multisamples
  end type mjuiThemeSpacing


  !---------------------------------- mjuiThemeColor ------------------------------------------------

  type, bind(c) :: mjuiThemeColor          !! UI visualization theme color
    real(c_float)                 :: master(3)                !! master background
    real(c_float)                 :: thumb(3)                 !! scrollbar thumb
    real(c_float)                 :: secttitle(3)             !! section title
    real(c_float)                 :: sectfont(3)              !! section font
    real(c_float)                 :: sectsymbol(3)            !! section symbol
    real(c_float)                 :: sectpane(3)              !! section pane
    real(c_float)                 :: shortcut(3)              !! shortcut background
    real(c_float)                 :: fontactive(3)            !! font active
    real(c_float)                 :: fontinactive(3)          !! font inactive
    real(c_float)                 :: decorinactive(3)         !! decor inactive
    real(c_float)                 :: decorinactive2(3)        !! inactive slider color 2
    real(c_float)                 :: button(3)                !! button
    real(c_float)                 :: check(3)                 !! check
    real(c_float)                 :: radio(3)                 !! radio
    real(c_float)                 :: select(3)                !! select
    real(c_float)                 :: select2(3)               !! select pane
    real(c_float)                 :: slider(3)                !! slider
    real(c_float)                 :: slider2(3)               !! slider color 2
    real(c_float)                 :: edit(3)                  !! edit
    real(c_float)                 :: edit2(3)                 !! edit invalid
    real(c_float)                 :: cursor(3)                !! edit cursor
  end type mjuiThemeColor


  !---------------------------------- mjuiItem ------------------------------------------------------

  type, bind(c) :: mjuiItemSingle          !! check and button-related
    integer(c_int)                :: modifier                   !! 0: none 1: control 2: shift 4: alt
    integer(c_int)                :: shortcut                   !! shortcut key 0: undefined
  end type mjuiItemSingle


  type, bind(c) :: mjuiItemMulti           !! static radio and select-related
    integer(c_int)                :: nelem                      !! number of elements in group
    character(len=1, kind=c_char), dimension(mjMAXUINAME, mjMAXUIMULTI) :: name !! element names
  end type mjuiItemMulti


  type, bind(c) :: mjuiItemSlider          !! slider-related
    real(c_double)                :: range(2)                !! slider range
    real(c_double)                :: divisions               !! number of range divisions
  end type mjuiItemSlider


  type, bind(c) :: mjuiItemEdit            !! edit-related
    integer(c_int)                :: nelem                   !! number of elements in list
    real(c_double)                :: range(2, mjMAXUIEDIT)   !! element range (min>=max: ignore)
  end type mjuiItemEdit


  type, bind(c) :: mjuiItemSingle_
    type(mjuiItemSingle) :: single !! check and button
  end type mjuiItemSingle_
  
  type, bind(c) :: mjuiItemMulti_
    type(mjuiItemMulti) :: multi  !! static, radio and select
  end type mjuiItemMulti_

  type, bind(c) :: mjuiItemSlider_
    type(mjuiItemMulti) :: slider !! slider
  end type mjuiItemSlider_

  type, bind(c) :: mjuiItemEdit_
    type(mjuiItemMulti) :: edit !! edit
  end type mjuiItemEdit_

  type, bind(c) :: mjuiItem                !! UI item
    !! common properties
    integer(c_int)                :: type                       !! type (mjtItem)
    character(len=1, kind=c_char), dimension(mjMAXUINAME) :: name         !! name
    integer(c_int)                :: state                      !! 0: disable 1: enable 2+: use predicate
    type(c_ptr)                   :: pdata                    !! data pointer (type-specific)
    integer(c_int)                :: sectionid                  !! id of section containing item
    integer(c_int)                :: itemid                     !! id of item within section

    !! type-specific properties
    ! union {
    !   struct mjuiItemSingle_ single; // check and button
    !   struct mjuiItemMulti_ multi;   // static, radio and select
    !   struct mjuiItemSlider_ slider; // slider
    !   struct mjuiItemEdit_ edit;     // edit
    ! };
    type(mjuiItemSingle_)         :: mjuiItemSingle_  !! check and button
    type(mjuiItemMulti_)          :: multi            !! static, radio and select
    type(mjuiItemSlider_)         :: slider           !! slider
    type(mjuiItemEdit_)           :: edit             !! edit

    !! internal
    type(mjrRect)                  :: rect                   !! rectangle occupied by item
  end type mjuiItem


  !---------------------------------- mjuiSection ---------------------------------------------------

  type, bind(c) :: mjuiSection             !! UI section
    !! properties
    character(len=1, kind=c_char), dimension(mjMAXUINAME) :: name !! name
    integer(c_int)                :: state                      !! 0: closed 1: open
    integer(c_int)                :: modifier                   !! 0: none 1: control 2: shift 4: alt
    integer(c_int)                :: shortcut                   !! shortcut key 0: undefined
    integer(c_int)                :: nitem                      !! number of items in use
    type(mjuiItem)                :: item(mjMAXUIITEM)          !! preallocated array of items

    !! internal
    type(mjrRect)                 :: rtitle                 !! rectangle occupied by title
    type(mjrRect)                 :: rcontent               !! rectangle occupied by content
  end type mjuiSection


  !---------------------------------- mjUI ----------------------------------------------------------

  type, bind(c) :: mjUI                    !! entire UI
    !! constants set by user
    type(mjuiThemeSpacing)        :: spacing       !! UI theme spacing
    type(mjuiThemeColor)          :: color           !! UI theme color
    type(c_funptr)                :: predicate        !! callback to set item state programmatically
    type(c_ptr)                   :: userdata                 !! pointer to user data (passed to predicate)
    integer(c_int)                :: rectid                     !! index of this ui rectangle in mjuiState
    integer(c_int)                :: auxid                      !! aux buffer index of this ui
    integer(c_int)                :: radiocol                   !! number of radio columns (0 defaults to 2)

    !! UI sizes (framebuffer units)
    integer(c_int)                :: width                      !! width
    integer(c_int)                :: height                     !! current heigth
    integer(c_int)                :: maxheight                  !! height when all sections open
    integer(c_int)                :: scroll                     !! scroll from top of UI

    !! mouse focus
    integer(c_int)                :: mousesect                  !! 0: none -1: scroll otherwise 1+section
    integer(c_int)                :: mouseitem                  !! item within section
    integer(c_int)                :: mousehelp                  !! help button down: print shortcuts

    !! keyboard focus and edit
    integer(c_int)                :: editsect                   !! 0: none otherwise 1+section
    integer(c_int)                :: edititem                   !! item within section
    integer(c_int)                :: editcursor                 !! cursor position
    integer(c_int)                :: editscroll                 !! horizontal scroll
    character(len=1, kind=c_char), dimension(mjMAXUITEXT) :: edittext !! current text
    type(c_ptr)                   :: editchanged          !! pointer to changed edit in last mjui_event

    !! sections
    integer(c_int)                :: nsect                      !! number of sections in use
    type(mjuiSection)             :: sect(mjMAXUISECT)  !! preallocated array of sections
  end type mjUI


  !---------------------------------- mjuiDef -------------------------------------------------------

  type, bind(c) :: mjuiDef                 !! table passed to mjui_add()
    integer(c_int)                :: type                       !! type (mjtItem) -1: section
    character(len=1, kind=c_char), dimension(mjMAXUINAME) :: name !! name
    integer(c_int)                :: state                      !! state
    type(c_ptr)                   :: pdata                      !! pointer to data
    character(len=1, kind=c_char), dimension(mjMAXUITEXT) :: other  !! string with type-specific properties
  end type mjuiDef
  
end module mod_mjui