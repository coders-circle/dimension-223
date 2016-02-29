#include <stdinc.h>
#include <ui/MainWindow.h>
#include <shaders/DiffuseMaterial.h>


MainWindow::MainWindow(int width, int height)
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

    m_window.add(m_widget);

    // Initialize glew once after SFMl widget is created
    glewInit();
}

MainWindow::~MainWindow()
{
    delete m_mesh;
    delete m_material;
    delete m_texture;
}

void MainWindow::OnInit()
{
    glEnable(GL_DEPTH_TEST);

    m_mesh = new Mesh3D(glm::vec3(1.0f));
    m_texture = new Texture("images/test.jpg");
    m_material = new DiffuseMaterial(glm::vec4(1), m_texture);
}

void MainWindow::OnDraw()
{
    glClearColor(0.396f, 0.612f, 0.937f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // test drawing with a 3d cube
    static float angle = 0;
    angle += 0.01f;

    glm::mat4 model =
        glm::translate(glm::mat4(), glm::vec3(0,-0.6f,-7)) *
        glm::rotate(glm::mat4(), angle, glm::vec3(0,1,0));
    m_mesh->Render(m_material, model, m_viewProjection);

    m_widget.Invalidate();
}

void MainWindow::OnResize(int width, int height)
{
    glViewport(0, 0, width, height);
    m_viewProjection = glm::perspective(120.f, float(width)/height,
        0.1f, 1000.0f);
}
