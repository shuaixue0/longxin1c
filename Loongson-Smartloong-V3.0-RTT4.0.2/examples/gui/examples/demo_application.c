#include <rtgui/rtgui.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/rtgui_app.h>

#include <rtgui/widgets/window.h>
#include <rtgui/widgets/notebook.h>

struct rtgui_notebook *the_notebook;

extern void demo_view_next(struct rtgui_object *object, struct rtgui_event *event);
extern void demo_view_prev(struct rtgui_object *object, struct rtgui_event *event);
extern rtgui_container_t *demo_view_animation();
extern rtgui_container_t *demo_view_benchmark(void);
extern rtgui_container_t *demo_view_bmp(void);
extern rtgui_container_t *demo_view_box(void);
extern struct rtgui_container *demo_view_buffer_animation(void);
extern rtgui_container_t *demo_view_button(void);
extern rtgui_container_t *demo_view_checkbox(void);
extern rtgui_container_t *demo_view_combobox(void);
extern rtgui_container_t *demo_view_dc(void);
extern rtgui_container_t *demo_view_dc_buffer();
extern rtgui_container_t *demo_view_edit(void);
extern rtgui_container_t *demo_view_image(void);
extern rtgui_container_t *demo_view_instrument_panel(void);
extern rtgui_container_t *demo_view_label(void);
extern rtgui_container_t *demo_view_listbox(void);
extern rtgui_container_t *demo_view_listctrl(void);
extern rtgui_container_t *demo_view_menu(void);
extern rtgui_container_t *demo_view_mywidget(void);
extern rtgui_container_t *demo_view_notebook(void);
extern rtgui_container_t *demo_view_progressbar(void);
extern rtgui_container_t *demo_view_radiobox(void);
extern rtgui_container_t *demo_view_scrollbar(void);
extern rtgui_container_t *demo_view_slider(void);
extern rtgui_container_t *demo_view_textbox(void);
extern rtgui_container_t *demo_view_ttf(void );
extern rtgui_container_t *demo_view_window(void);
extern struct rtgui_container* demo_plot(void);
extern rtgui_container_t * demo_view_digtube(void);  


static rt_bool_t demo_handle_key(struct rtgui_object *object, struct rtgui_event *event)
{
    struct rtgui_event_kbd *ekbd = (struct rtgui_event_kbd *)event;

    if (ekbd->type == RTGUI_KEYUP)
    {
        if (ekbd->key == RTGUIK_RIGHT)
        {
            demo_view_next(RT_NULL, RT_NULL);
            return RT_TRUE;
        }
        else if (ekbd->key == RTGUIK_LEFT)
        {
            demo_view_prev(RT_NULL, RT_NULL);
            return RT_TRUE;
        }
    }
    return RT_TRUE;
}

struct rtgui_win *main_win;
static void application_entry(void *parameter)
{
    struct rtgui_app *app;
    struct rtgui_rect rect;

    app = rtgui_app_create("gui_demo");
    if (app == RT_NULL)
        return;
    /* create a full screen window */
    rtgui_graphic_driver_get_rect(rtgui_graphic_driver_get_default(), &rect);

    main_win = rtgui_win_create(RT_NULL, "demo_win", &rect,
                                RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
    if (main_win == RT_NULL)
    {
        rtgui_app_destroy(app);
        return;
    }

    rtgui_win_set_onkey(main_win, demo_handle_key);

    /* create a no title notebook that we can switch demo on it easily. */
    the_notebook = rtgui_notebook_create(&rect, RTGUI_NOTEBOOK_NOTAB);
    if (the_notebook == RT_NULL)
    {
        rtgui_win_destroy(main_win);
        rtgui_app_destroy(app); 
        return;
    }

    rtgui_container_add_child(RTGUI_CONTAINER(main_win), RTGUI_WIDGET(the_notebook));

    //demo_view_box();

    /* ???????????????????? */
    demo_view_benchmark();

    demo_view_dc();
#ifdef RTGUI_USING_TTF
    demo_view_ttf();
#endif

#ifndef RTGUI_USING_SMALL_SIZE
    demo_view_dc_buffer();
#endif
    demo_view_animation();
#ifndef RTGUI_USING_SMALL_SIZE
    demo_view_buffer_animation();
    demo_view_instrument_panel();
#endif
    demo_view_window();
    demo_view_label();
    demo_view_button();
    demo_view_checkbox();
    demo_view_progressbar();
    demo_view_scrollbar();
    demo_view_radiobox();
    demo_view_textbox();
    demo_view_listbox();
    demo_view_menu();
    demo_view_listctrl();
    demo_view_combobox();
    demo_view_slider();
    demo_view_notebook();
    demo_view_mywidget();
    demo_plot();
	demo_view_digtube();

#if defined(RTGUI_USING_DFS_FILERW)
	demo_view_edit();
	demo_view_bmp();
#endif

#if 0
#if defined(RTGUI_USING_DFS_FILERW)
    demo_view_image();
#endif
#ifdef RT_USING_MODULE
#if defined(RTGUI_USING_DFS_FILERW)
    demo_view_module();
#endif
#endif
    demo_listview_view();
    demo_listview_icon_view();
#if defined(RTGUI_USING_DFS_FILERW)
    demo_fn_view();
#endif
#endif

    rtgui_win_show(main_win, RT_FALSE);

    /* ?????????????????? */
    rtgui_app_run(app);

    rtgui_app_destroy(app);
}

void application_init()
{
    static rt_bool_t inited = RT_FALSE;

    if (inited == RT_FALSE) /* ???????????????????????? */
    {
        rt_thread_t tid;

        tid = rt_thread_create("wb",
                               application_entry, RT_NULL,
                               2048 * 2, 25, 10);

        if (tid != RT_NULL)
            rt_thread_startup(tid);

        inited = RT_TRUE;
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void application()
{
    application_init();
}
/* finsh????????????????????????application()???????????????????? */
FINSH_FUNCTION_EXPORT(application, application demo)
#endif
