module mod_mj
  use iso_c_binding, only : C_INT, C_CHAR, C_NULL_CHAR, C_PTR, c_f_pointer
  use mod_mjrender
  implicit none
  
  interface

    function mj_load_xml(filename, vfs, error, error_sz) result(model) bind(C, name="mj_loadXML")
      import :: c_char, c_ptr, c_int
      character(c_char), intent(IN)      :: filename(*)
      type(c_ptr), value                :: vfs
      character(c_char), intent(INOUT)   :: error(*)
      integer(C_INT), value, intent(IN) :: error_sz
      type(c_ptr)                       :: model
    end function mj_load_xml

    function mj_make_data(m) result(data) bind(C, name="mj_makeData")
      import :: C_PTR
      type(c_ptr), value :: m
      type(c_ptr) :: data
    end function mj_make_data

    subroutine mjv_default_camera(cam) bind(C, name="mjv_defaultCamera")
      import :: c_ptr
      type(c_ptr), intent(in), value :: cam
    end subroutine mjv_default_camera

    subroutine mjv_default_option(opt) bind(C, name="mjv_defaultOption")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: opt
    end subroutine mjv_default_option

    subroutine mjv_default_scene(scn) bind(C, name="mjv_defaultScene")
      import :: c_ptr
      type(c_ptr), intent(in), value :: scn
    end subroutine mjv_default_scene

    subroutine mjr_default_context(con) bind(C, name="mjr_defaultContext")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: con
    end subroutine mjr_default_context

    subroutine mjv_make_scene(m, scn, maxgeom) bind(C, name="mjv_makeScene")
      import :: c_ptr, C_INT
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: scn
      integer(C_INT), value, intent(IN) :: maxgeom
    end subroutine mjv_make_scene

    subroutine mjr_make_context(m, con, fontscale) bind(C, name="mjr_makeContext")
      import :: C_INT, c_ptr
      implicit none
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: con
      integer(C_INT), value, intent(IN) :: fontscale
    end subroutine mjr_make_context

    subroutine mj_step(m, d) bind(C, name="mj_step")
      import :: c_ptr
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
    end subroutine mj_step

    subroutine mjv_update_scene(m, d, opt, pert, cam, catmask, scn) &
          bind(C, name="mjv_updateScene")
      import :: c_ptr, c_int
      implicit none
      type(c_ptr), intent(IN), value :: m
      type(c_ptr), value :: d
      type(c_ptr), intent(IN), value :: opt
      type(c_ptr), intent(IN), value :: pert
      type(c_ptr), value :: cam
      integer(C_INT), value, intent(IN) :: catmask
      type(c_ptr), value :: scn
    end subroutine mjv_update_scene

    subroutine mjr_render(viewport, scn, con) bind(C, name="mjr_render")
        import :: c_ptr, mjrRect
        type(mjrRect), value, intent(IN) :: viewport
        type(c_ptr), value :: scn
        type(c_ptr), intent(IN), value :: con
      end subroutine mjr_render

    subroutine mju_error(msg) bind(C, name="mju_error")
      import :: C_CHAR
      character(kind=C_CHAR), intent(IN) :: msg(*)
    end subroutine mju_error

    subroutine mju_error_s(msg, text) bind(C, name="mju_error_s")
      import :: C_CHAR
      character(kind=C_CHAR), intent(IN) :: msg(*)
      character(kind=C_CHAR), intent(IN) :: text(*)
    end subroutine mju_error_s

  end interface
contains
  
end module mod_mj