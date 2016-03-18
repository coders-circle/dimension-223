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
}


Renderer::~Renderer()
{
    delete m_worldMesh;
    delete m_worldMaterial;
}


void Renderer::OnInit()
{
    // Initialize glew once after SFMl widget is created
    glewInit();

    glEnable(GL_DEPTH_TEST);

    m_worldMesh = new Mesh(glm::vec3(5.f, 5.f, 0.1f));
    m_worldMaterial = new DepthMaterial();
}


void Renderer::OnDraw()
{
    glClearColor(0.396f, 0.612f, 0.937f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 viewProjection = m_camera.GetViewProjection();

    glm::mat4 world =  glm::translate(glm::mat4(), glm::vec3(0, 0, -5));
    m_worldMesh->Render(m_worldMaterial, world, viewProjection);

    m_widget.Invalidate();
}


void Renderer::OnResize(int width, int height)
{
    glViewport(0, 0, width, height);
    // m_viewProjection = glm::perspective(120.f, float(width)/height,
    //     0.1f, 1000.0f);
}
