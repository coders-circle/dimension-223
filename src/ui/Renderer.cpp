#include <stdinc.h>
#include <ui/Renderer.h>


Renderer::Renderer(Gtk::Window& parent, int width, int height)
    : m_widget(width, height)
{
    m_widget.SetDrawCallback([this]()
    {
        OnDraw();
    });

    m_widget.SetResizeCallback([this](int w, int h)
    {
        OnResize(w, h);
    });

    m_widget.SetInitCallback([this]()
    {
        OnInit();
    });

    m_widget.show();

    parent.add(m_widget);

    // Initialize glew once after SFMl widget is created
    glewInit();
}


void Renderer::OnInit()
{
    glEnable(GL_DEPTH_TEST);
}


void Renderer::OnDraw()
{
    glClearColor(0.396f, 0.612f, 0.937f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_widget.Invalidate();
}


void Renderer::OnResize(int width, int height)
{
    glViewport(0, 0, width, height);
    // m_viewProjection = glm::perspective(120.f, float(width)/height,
    //     0.1f, 1000.0f);
}
