program main_mujoco_test
  use iso_c_binding
  use mod_mujoco
  use mod_glfw
  implicit none
  
  ! MuJoCo data structures
  type(mjModel), pointer  :: m                    !! MuJoCo model
  type(mjData), pointer   :: d                    !! MuJoCo data
  type(mjvCamera)         :: cam                  !! abstract camera
  type(mjvOption)         :: opt                  !! visualization options
  type(mjvScene)          :: scn                  !! abstract scene
  type(mjrContext)        :: con                  !! custom GPU context

  logical(c_bool)         :: button_left    = .false._c_bool
  logical(c_bool)         :: button_middle  = .false._c_bool
  logical(c_bool)         :: button_right   = .false.
  real(c_double)          :: lastx          = 0.0_c_double
  real(c_double)          :: lasty          = 0.0_c_double

  ! ! holders of one step history of time and position to calculate dertivatives
  ! real(mjtNum)            :: position_history = 0
  ! real(mjtNum)            :: previous_time    = 0

  ! ! controller related variables
  ! real(c_float)           :: ctrl_update_freq = 100
  ! real(mjtNum)            :: last_update = 0.0
  ! real(mjtNum)            :: ctrl
  
  
  character(len=1, kind=c_char) :: loaderror(1000) = "Could not load binary model"//c_null_char
  integer(c_int), parameter :: error_sz = 1000_c_int
  
  type(GLFWwindow), pointer :: window
  integer(c_int) :: glfw_init, close_window

  type(c_ptr)               :: model_ptr = c_null_ptr
  type(c_ptr)               :: data_ptr = c_null_ptr
  type(c_ptr)               :: window_ptr = c_null_ptr

  type(c_funptr)            :: key_callback, keyboard_funptr
  type(c_funptr)            :: cursor_pos_callback, cursor_pos_funptr
  type(c_funptr)            :: mouse_button_callback, mouse_button_funptr
  type(c_funptr)            :: scroll_callback, scroll_funptr

  real(mjtNum)              :: simstart
  type(mjrRect)             :: viewport

  ! load and compile model
  !!!!!! ADD THE PATH TO ball.xml IN THE SRC/ASSETS FOLDER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  model_ptr = mj_loadXML(&
            filename="CHANGE_THIS_TO_PATH_TO_BALL.XML_IN_SRC/ASSETS_FOLDER/ball.xml"//c_null_char, &
            error=loaderror, error_sz=error_sz)
  if ( .not. c_associated(model_ptr) ) call mju_error_s("Load model error: %s"//c_null_char, loaderror)
  call c_f_pointer(model_ptr, m)

  data_ptr = mj_makeData(m)
  if ( .not. c_associated(data_ptr) ) call mju_error_s("Load data error: %s"//c_null_char, loaderror)
  call c_f_pointer(data_ptr, d)

  glfw_init = glfwInit()
  if( glfw_init == GLFW_FALSE ) call mju_error("Could not initialize GLFW"//c_null_char)

  ! Create window, make OpenGL context current, request v-sync
  window_ptr = glfwCreateWindow(width=1200, height=900, title="Demo"//c_null_char)
  call c_f_pointer(window_ptr, window)
  call glfwMakeContextCurrent(window)
  call glfwSwapInterval(1)

  ! ! Initialize visualization data structures
  call mjv_defaultCamera(cam)
  call mjv_defaultOption(opt)
  call mjv_defaultScene(scn)
  call mjr_defaultContext(con)

  ! create scene and context
  call mjv_makeScene(m, scn, 2000)                !! space for 2000 objects
  call mjr_makeContext(m, con, mjFONTSCALE_150)  !! model-specific context

  ! Install GLFW mouse and keyboard callbacks
  keyboard_funptr = c_funloc(keyboard)
  key_callback = glfwSetKeyCallback(window, keyboard_funptr)
  
  ! ! cursor_proc_ptr => mouse_move
  ! ! tmp_funptr = glfwSetCursorPosCallback(window, cursor_proc_ptr)
  cursor_pos_funptr = c_funloc(mouse_move)
  cursor_pos_callback = glfwSetCursorPosCallback(window, cursor_pos_funptr)

  ! ! mousebutton_proc_ptr => mouse_button
  mouse_button_funptr = c_funloc(mouse_button)
  mouse_button_callback = glfwSetMouseButtonCallback(window, mouse_button_funptr)

  ! ! scroll_proc_ptr => scroll
  scroll_funptr = c_funloc(scroll)
  scroll_callback = glfwSetScrollCallback(window, scroll_funptr)

  do while( glfwWindowShouldClose(window) == 0)
    !! advance interactive simulation for 1/60 sec
    !!  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    !!  this loop will finish on time for the next frame to be rendered at 60 fps.
    !!  Otherwise add a cpu timer and exit this loop when it is time to render.
    simstart = d%time

    sim_loop: do
      ! print *, "d%time - simstart = ", d%time - simstart
      if ( (d%time - simstart) >= (1.0/60.0) ) exit sim_loop
      call mj_step(m, d)
    end do sim_loop

    !! get framebuffer viewport
    viewport = mjrRect(0_c_int, 0_c_int, 0_c_int, 0_c_int)
    call glfwGetFramebufferSize(window, viewport%width, viewport%height)

  !   !! update scene and render
    call mjv_updateScene(m=m, d=d, opt=opt, cam=cam, cammask=mjCAT_ALL, scn=scn)
    call mjr_render(viewport, scn, con)

  !   !! swap OpenGL buffers (blocking call due to v-sync)
    call glfwSwapBuffers(window)

    !! process pending GUI events, call GLFW callbacks
    call glfwPollEvents();
  end do

  ! free visualization storage
  call mjv_freeScene(scn)
  call mjr_freeContext(con)

  !! free MuJoCo model and data, deactivate
  call mj_deleteData(d)
  call mj_deleteModel(m)
  call mj_deactivate()

contains

  ! keyboard callback
  subroutine keyboard(input_window, key, scancode, act, mods) bind(c)
    type(GLFWwindow), intent(inout)   :: input_window
    integer(c_int), value, intent(in) :: key, scancode, act, mods

    !! backspace: reset simulation
    if( (act == GLFW_PRESS) .and. (key == GLFW_KEY_BACKSPACE) ) then
      call mj_resetData(m, d)
      call mj_forward(m, d)
    end if
  end subroutine keyboard
  
  ! mouse button callback
  subroutine mouse_button(input_window, button, act, mods)
    type(GLFWwindow), intent(inout)   :: input_window
    integer(c_int), value, intent(in) :: button, act, mods

    !! update button state
    button_left   = ( glfwGetMouseButton(input_window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS  )
    button_middle = ( glfwGetMouseButton(input_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS  )
    button_right  = ( glfwGetMouseButton(input_window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS  )

    !! update mouse position
    call glfwGetCursorPos(input_window, lastx, lasty)
  end subroutine mouse_button


  ! mouse move callback
  subroutine mouse_move(input_window, xpos, ypos)
    type(GLFWwindow), intent(inout)      :: input_window
    real(c_double), value, intent(in) :: xpos, ypos

    real(c_double)  :: dx, dy
    integer(c_int)  :: width, height
    logical(c_bool) :: mod_shift

    integer(c_int)  :: action

    !! no buttons down: nothing to do
    if( (button_left .eqv. .false.) .and. (button_middle .eqv. .false.) .and. (button_right .eqv. .false.) ) return

    !! compute mouse displacement, save
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    !! get current window size
    call glfwGetWindowSize(input_window, width, height)

    !! get shift key state
    mod_shift = ( (glfwGetKey(input_window, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS)    &
                  .or.                                                        &
                  (glfwGetKey(input_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) )

    !! determine action based on mouse button
    if( button_right ) then
      action = merge( mjMOUSE_MOVE_H, mjMOUSE_MOVE_V, mod_shift .eqv. .true. )
    else if( button_left ) then
      action = merge( mjMOUSE_ROTATE_H, mjMOUSE_ROTATE_V, mod_shift .eqv. .true.)
    else
      action = mjMOUSE_ZOOM
    end if

    !! move camera
    call mjv_moveCamera(m, action, dx/height, dy/height, scn, cam)
  end subroutine mouse_move

  ! scroll callback
  subroutine scroll(input_window, xoffset, yoffset)
    type(GLFWwindow), intent(inout)   :: input_window
    real(c_double), value, intent(in) :: xoffset, yoffset
      
    !! emulate vertical mouse motion = 5% of window height
    call mjv_moveCamera(m, mjMOUSE_ZOOM, 0.0_c_double, -0.05*yoffset, scn, cam)
  end subroutine scroll
end program main_mujoco_test