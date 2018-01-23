#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <iostream>
#include <chrono/physics/ChBodyEasy.h>

NGLScene::NGLScene()
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  setTitle("Chrono NGL");

}


void NGLScene::buildWorld()
{
  chrono::collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.3);
  for (int bi = 0; bi < 400; bi++)
  {
    // Create a bunch of ChronoENGINE rigid bodies which will fall..
    auto mrigidBody = std::make_shared<chrono::ChBodyEasySphere>(0.81,   // radius
                                                         1000,   // density
                                                         true,   // collide enable?
                                                         true);  // visualization?
    mrigidBody->SetPos(chrono::ChVector<>(-5 + chrono::ChRandom() * 10, 4 + bi * 0.05, -5 + chrono::ChRandom() * 10));
    mrigidBody->GetMaterialSurfaceNSC()->SetFriction(0.3f);

    m_physicalSystem.Add(mrigidBody);
    }
    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    auto floorBody = std::make_shared<chrono::ChBodyEasyBox>(20, 1, 20,  // x,y,z size
                                                     1000,       // density
                                                     true,       // collide enable?
                                                     true);      // visualization?
    floorBody->SetPos(chrono::ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);

    m_physicalSystem.Add(floorBody);

    auto wallBody1 = std::make_shared<chrono::ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody1->SetPos(chrono::ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);

    m_physicalSystem.Add(wallBody1);

    auto wallBody2 = std::make_shared<chrono::ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody2->SetPos(chrono::ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);

    m_physicalSystem.Add(wallBody2);

    auto wallBody3 = std::make_shared<chrono::ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody3->SetPos(chrono::ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);

    m_physicalSystem.Add(wallBody3);

    auto wallBody4 = std::make_shared<chrono::ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody4->SetPos(chrono::ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);

    m_physicalSystem.Add(wallBody4);

    // Add the rotating mixer
    auto rotatingBody = std::make_shared<chrono::ChBodyEasyBox>(10, 5, 1,  // x,y,z size
                                                        4000,      // density
                                                        true,      // collide enable?
                                                        true);     // visualization?
    rotatingBody->SetPos(chrono::ChVector<>(0, -1.6, 0));
    rotatingBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    rotatingBody->SetName("mixer");
    m_physicalSystem.Add(rotatingBody);

    // .. an engine between mixer and truss
    auto my_motor = std::make_shared<chrono::ChLinkEngine>();
    my_motor->Initialize(rotatingBody, floorBody, chrono::ChCoordsys<>(chrono::ChVector<>(0, 0, 0), Q_from_AngAxis(chrono::CH_C_PI_2, chrono::VECT_X)));
    my_motor->Set_eng_mode(chrono::ChLinkEngine::ENG_MODE_SPEED);
    if (auto mfun = std::dynamic_pointer_cast<chrono::ChFunction_Const>(my_motor->Get_spe_funct()))
        mfun->Set_yconst(chrono::CH_C_PI / 2.0);  // speed w=90/s
    m_physicalSystem.AddLink(my_motor);

}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
  m_physicalSystem.Clear();

}



void NGLScene::resizeGL(int _w , int _h)
{
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
  m_projection=ngl::perspective( 45.0f, static_cast<float>( _w ) / _h, 0.05f, 350.0f );

}


void NGLScene::initializeGL()
{
  // we need to initialise the NGL lib which will load all of the OpenGL functions, this must
  // be done once we have a valid GL context but before we call any GL commands. If we dont do
  // this everything will crash
  ngl::NGLInit::instance();
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);
  chrono::GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
  std::cout<<"building world\n";
  buildWorld();
  m_physicalSystem.SetSolverType(chrono::ChSolver::Type::SOR);
  m_physicalSystem.SetMaxItersSolverSpeed(20);
  m_contactCallback.msystem = &m_physicalSystem;  // will be used by callback
  m_physicalSystem.GetContactContainer()->RegisterAddContactCallback(&m_contactCallback);
  std::cout<<"starting timers\n";
  startTimer(10);

  // now to load the shader and set the values
  // grab an instance of shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  (*shader)["nglDiffuseShader"]->use();
  shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
  shader->setUniform("lightPos",1.0f,1.0f,1.0f);
  shader->setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  prim->createSphere("sphere",1.0f,60.0f);
  prim->createLineGrid("plane",140.0f,140.0f,40.0f);
  m_view = ngl::lookAt({20,20,20},{0,0,0},{0,1,0});
  m_projection=ngl::perspective( 45.0f, static_cast<float>( width() ) / height(), 0.05f, 350.0f );


}

void NGLScene::timerEvent(QTimerEvent *)
{
  if(!m_pause)
    m_physicalSystem.DoStepDynamics(0.05);
  update();

}


void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;
  M  = m_globalTransformMatrix*m_tx.getMatrix();
  MV = m_view*M;
  MVP= m_projection*MV;

  normalMatrix=MV;

  normalMatrix.inverse().transpose();
  shader->setUniform("MVP",MVP);
  shader->setUniform("normalMatrix",normalMatrix);
}


void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_win.width,m_win.height);
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  // Rotation based on the mouse position for our global transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_win.spinXFace);
  rotY.rotateY(m_win.spinYFace);
  // multiply the rotations
  m_globalTransformMatrix=rotY*rotX;
  // add the translations
  m_globalTransformMatrix.m_m[3][0] = m_modelPos.m_x;
  m_globalTransformMatrix.m_m[3][1] = m_modelPos.m_y;
  m_globalTransformMatrix.m_m[3][2] = m_modelPos.m_z;

  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);

  for (size_t i = 0; i < m_physicalSystem.Get_bodylist()->size(); i++)
  {
     auto abody = m_physicalSystem.Get_bodylist()->at(i);
     for (int i = 0; i < abody->GetAssets().size(); i++)
     {
             auto asset = abody->GetAssets().at(i);
             const chrono::Vector pos = abody->GetFrame_REF_to_abs().GetPos();
           //rotation of the body
             chrono::Quaternion rot = abody->GetFrame_REF_to_abs().GetRot();
             m_tx.setPosition(pos.x(),pos.y(),pos.z());
             chrono::Vector r=rot.Q_to_Euler123();
             m_tx.setRotation(ngl::degrees(r.x()),ngl::degrees(r.y()),ngl::degrees(r.z()));

           if (chrono::ChSphereShape* sphere_shape = dynamic_cast<chrono::ChSphereShape*>(asset.get()))
           {
               double radius = sphere_shape->GetSphereGeometry().rad;
               m_tx.setScale(radius,radius,radius);
               shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
               loadMatricesToShader();
               prim->draw("sphere");
            }
           else if (chrono::ChBoxShape* box_shape = dynamic_cast<chrono::ChBoxShape*>(asset.get()))
           {
              chrono::Vector scale = box_shape->GetBoxGeometry().Size;
              m_tx.setScale(scale.x()*2,scale.y()*2,scale.z()*2);
              loadMatricesToShader();
              if(strcmp(abody->GetName(),"mixer")==0)
                shader->setUniform("Colour",1.0f,0.0f,0.0f,1.0f);
              else
                shader->setUniform("Colour",0.0f,1.0f,0.0f,1.0f);
              prim->draw("cube");
            }
      }
  }


}

//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  case Qt::Key_Space :
      m_win.spinXFace=0;
      m_win.spinYFace=0;
      m_modelPos.set(ngl::Vec3::zero());

  break;
  case Qt::Key_P : m_pause ^=true; break;
  default : break;
  }
  // finally update the GLWindow and re-draw

    update();
}
