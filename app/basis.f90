program basis
  use iso_c_binding
  use mod_glfw
  use mod_mujoco
  ! use mod_mjdata
  ! use mod_mjvisualize
  ! use mod_mjrender
  implicit none
  
  character(len=1, kind=c_char) :: loaderror(1000) = "Could not load binary model"//c_null_char
  character(len=*, kind=c_char), parameter :: filename &
                    ="path_to_ball.xml/ball.xml"//c_null_char
                    
  type(c_ptr)                     :: m, d, window
  type(mjvCamera), target         :: cam
  type(mjvOption), target         :: opt
  type(mjvScene), target          :: scn
  type(mjrContext), target        :: con

  type(mjData), pointer           :: data
  type(mjrRect)                   :: viewport

  real(c_double)                  :: sim_start

  logical(c_bool)                 :: button_left    = .false._c_bool
  logical(c_bool)                 :: button_middle  = .false._c_bool
  logical(c_bool)                 :: button_right   = .false._c_bool
  real(c_double)                  :: lastx          = 0.0_c_double
  real(c_double)                  :: lasty          = 0.0_c_double

  ! procedure(GLFWkeyfun), pointer           :: keyboard_callback     => null()
  ! procedure(GLFWcursorposfun), pointer     :: cursor_callback       => null()
  ! procedure(GLFWmousebuttonfun), pointer   :: mousebutton_callback  => null()
  ! procedure(GLFWscrollfun), pointer        :: scroll_callback       => null()

  type(c_funptr)                  :: keyboard_funptr, cursor_funptr, mouse_button_funptr, scroll_funptr
  

  m = mj_loadXML( filename, c_null_ptr, loaderror, 1000 )
  if (.not. c_associated(m) ) call mju_error_s("Load model error: %s"//c_null_char, loaderror)

  d = mj_makeData(m)
  if ( .not. c_associated(d) ) call mju_error_s("Load data error: %s"//c_null_char, loaderror)

  if (glfwInit() == GLFW_FALSE) call mju_error("Could not initialize GLFW")

  window = glfwCreateWindow(1200, 900, "Demo"//c_null_char, c_null_ptr, c_null_ptr)
  if ( .not. c_associated(window) ) call mju_error_s("Could not create GLFW Window!!!"//c_null_char, loaderror)
  
  call glfwMakeContextCurrent( window )
  call glfwSwapInterval( 1_c_int )

  ! Initialize visualization data structures
  call mjv_defaultCamera( c_loc(cam) )
  call mjv_defaultOption( c_loc(opt) )
  call mjv_defaultScene( c_loc(scn) )
  call mjr_defaultContext( c_loc(con) )

  ! Create scene and context
  call mjv_makeScene(m, c_loc(scn), 2000);
  call mjr_makeContext(m, c_loc(con), mjFONTSCALE_150);

  call c_f_pointer(d, data)

  ! Install GLFW mouse and keyboard callbacks
  ! keyboard_callback     => keyboard
  ! cursor_callback       => mouse_move
  ! mousebutton_callback  => mouse_button
  ! scroll_callback       => scroll

  keyboard_funptr     = glfwSetKeyCallback( window, keyboard )
  cursor_funptr       = glfwSetCursorPosCallback( window, mouse_move )
  mouse_button_funptr = glfwSetMouseButtonCallback( window, mouse_button )
  scroll_funptr       = glfwSetScrollCallback( window, scroll )

  do while ( glfwWindowShouldClose(window) == 0)
    sim_start = data%time
    do while ( (data%time - sim_start) < (1.0/60.0) )
      call mj_step( m, d )
    end do

    ! Get framebuffer viewport
    viewport = mjrRect( 0_c_int, 0_c_int, 0_c_int, 0_c_int )
    call glfwGetFramebufferSize( window, viewport%width, viewport%height )

    ! Update scene and render
    call mjv_updateScene( m, d, c_loc(opt), c_null_ptr, c_loc(cam), mjCAT_ALL, c_loc(scn) )

    call mjr_render( viewport, c_loc(scn), c_loc(con) )

    call glfwSwapBuffers( window )

    call glfwPollEvents()
  end do

  ! Free visualization storage
  call mjv_freeScene( c_loc(scn) )
  call mjr_freeContext( c_loc(con) )

  ! Free MuJoCo model and data
  call mj_deleteData( d )
  call mj_deleteModel( m )

contains

  ! keyboard callback
  subroutine keyboard( input_window, key, scancode, act, mods ) bind(c)
    type(c_ptr), value                :: input_window
    integer(c_int), value, intent(in) :: key, scancode, act, mods

    !! backspace: reset simulation
    if( (act == GLFW_PRESS) .and. (key == GLFW_KEY_BACKSPACE) ) then
      call mj_resetData( m, d )
      call mj_forward( m , d )
    end if
  end subroutine keyboard


  ! mouse button callback
  subroutine mouse_button( input_window, button, act, mods ) bind(c)
    type(c_ptr), value                :: input_window
    integer(c_int), value, intent(in) :: button, act, mods

    !! Update button state
    button_left   = ( glfwGetMouseButton(input_window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS  )
    button_middle = ( glfwGetMouseButton(input_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS  )
    button_right  = ( glfwGetMouseButton(input_window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS  )

    !! Update mouse position
    call glfwGetCursorPos( input_window, lastx, lasty )
  end subroutine mouse_button


  ! mouse move callback
  subroutine mouse_move( input_window, xpos, ypos ) bind(c)
    type(c_ptr), value                :: input_window
    real(c_double), value, intent(in) :: xpos, ypos

    real(c_double)  :: dx, dy
    integer(c_int)  :: width, height
    logical(c_bool) :: mod_shift

    integer(c_int)  :: action

    !! No buttons down: nothing to do
    if( (button_left .eqv. .false.) .and. (button_middle .eqv. .false.) .and. (button_right .eqv. .false.) ) return

    !! Compute mouse displacement, save
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    !! Get current window size
    call glfwGetWindowSize(input_window, width, height)

    !! Get shift key state
    mod_shift = ( (glfwGetKey(input_window, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS)    &
                  .or.                                                              &
                  (glfwGetKey(input_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) )

    !! Determine action based on mouse button
    if ( button_right ) then
      action = merge( mjMOUSE_MOVE_H, mjMOUSE_MOVE_V, mod_shift .eqv. .true. )
    else if( button_left ) then
      action = merge( mjMOUSE_ROTATE_H, mjMOUSE_ROTATE_V, mod_shift .eqv. .true.)
    else
      action = mjMOUSE_ZOOM
    end if

    !! move camera
    call mjv_moveCamera( m, action, (dx/height), (dy/height), c_loc(scn), c_loc(cam) )
  end subroutine mouse_move

  ! scroll callback
  subroutine scroll( input_window, xoffset, yoffset ) bind(c)
    type(c_ptr), value                :: input_window
    real(c_double), value, intent(in) :: xoffset, yoffset
      
    !! emulate vertical mouse motion = 5% of window height
    call mjv_moveCamera( m, mjMOUSE_ZOOM, 0.0_c_double, -0.05*yoffset, c_loc(scn), c_loc(cam) )
  end subroutine scroll

end program basis