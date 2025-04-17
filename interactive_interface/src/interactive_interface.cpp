#include <memory>
#include <imgui.h>
#include <portable-file-dialogs.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <glk/lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <ros/package.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "../../odometry2graph/include/odometry2graph.h"

std::string data_directory;

/**
 * @brief Application to convert an odometry sequence into the graph description format
 *
 */
class Odometry2GraphApplication : public guik::Application
{
public:
  Odometry2GraphApplication() : Application() {}
  ~Odometry2GraphApplication() {}

  /**
   * @brief initialize the application
   *
   * @param size            window size
   * @param glsl_version    glsl version
   * @return if successfully initialized
   */
  bool init(const char *window_name, const Eigen::Vector2i &size, const char *glsl_version = "#version 330") override
  {
    if (!Application::init(window_name, size, glsl_version))
    {
      return false;
    }

    framebuffer_size = size;
    progress.reset(new guik::ProgressModal("progress modal"));

    main_canvas.reset(new guik::GLCanvas(data_directory, size));
    if (!main_canvas->ready())
    {
      close();
    }

    keyframe_delta_x = 3.0f;
    keyframe_delta_angle = 90.0f;
    downsample_resolution = 0.2f;
    is_downsample = false;
    friends_num = 0;

    last_keyframe_delta_x = keyframe_delta_x;
    last_keyframe_delta_angle = keyframe_delta_angle;
    last_downsample_resolution = downsample_resolution;
    last_is_downsample = is_downsample;
    last_friends_num = friends_num;

    return true;
  }

  /**
   * @brief draw ImGui-based UI
   *
   */
  virtual void draw_ui() override
  {
    main_canvas->draw_ui();

    {
      ImGui::Begin("keyframe settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::DragFloat("downsample_resolution", &downsample_resolution, 0.01f, 0.01f, 1.0f);
      ImGui::DragFloat("keyframe_delta_x", &keyframe_delta_x, 0.1f, 0.1f, 100.0f);
      ImGui::DragFloat("keyframe_delta_angle", &keyframe_delta_angle, 0.01f, 5.0f, 360.0f);
      ImGui::Checkbox("is_downsample", &is_downsample);
      ImGui::DragInt("friends_num", &friends_num, 1, 0, 50);

      if (ImGui::Button("Start"))
      {
        if (std::abs(last_keyframe_delta_x - keyframe_delta_x) > 0.01f ||
            std::abs(last_keyframe_delta_angle - keyframe_delta_angle) > 0.01f)
        {
          odometry_set->reset_keyframe_delta(keyframe_delta_x, keyframe_delta_angle);
          is_update = true;
        }

        if (std::abs(last_downsample_resolution - downsample_resolution) > 0.01f ||
            last_is_downsample != is_downsample)
        {
          odometry_set->reset_downsample(downsample_resolution, is_downsample);
          is_update = true;
        }

        if (last_friends_num != friends_num)
        {
          odometry_set->reset_friends_num(friends_num);
          is_update = true;
        }

        last_keyframe_delta_x = keyframe_delta_x;
        last_keyframe_delta_angle = keyframe_delta_angle;
        last_downsample_resolution = downsample_resolution;
        last_is_downsample = is_downsample;
        last_friends_num = friends_num;
      }

      // bool updated = false;
      // updated |= ImGui::DragFloat("keyframe_delta_x", &keyframe_delta_x, 0.1f, 0.1f, 100.0f);
      // updated |= ImGui::DragFloat("keyframe_delta_angle", &keyframe_delta_angle, 0.01f, 0.01f, 3.15f);

      // if (delta_updated && !updated)
      // {
      //   odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
      // }
      // delta_updated = updated;

      ImGui::End();
    }

    main_menu();
    mouse_control();
  }

  /**
   * @brief draw OpenGL-related things
   *
   */

  void update_cloud_buffer()
  {
    if (is_update == false)
      return;
    if (!odometry_set)
    {
      std::cout << "odometry_set not ready!" << std::endl;
      return;
    }

    all_clouds.clear();
    odometry_set->get_all_clouds(all_clouds);

    cloud_buffer.clear();

    // cloud_buffer = std::vector<std::unique_ptr<glk::PointCloudBuffer>>(all_clouds.size());

    for (int i = 0; i < all_clouds.size(); ++i)
    {
      cloud_buffer.push_back(std::make_unique<glk::PointCloudBuffer>(all_clouds[i].first));
    }

    is_update = false;
  }

  void draw_clouds()
  {
    update_cloud_buffer();
    for (int i = 0; i < cloud_buffer.size(); ++i)
    {
      // main_canvas->shader->set_uniform("color_mode", 0);
      // main_canvas->shader->set_uniform("model_matrix", all_clouds[i].second.cast<float>().matrix());
      // cloud_buffer[i]->draw(*main_canvas->shader);

      // main_canvas->shader->set_uniform("color_mode", 1);
      // main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
      // main_canvas->shader->set_uniform("apply_keyframe_scale", true);
      // auto &sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
      // sphere.draw(*main_canvas->shader);
      // main_canvas->shader->set_uniform("apply_keyframe_scale", false);

      glk::GLSLShader &shader = (*main_canvas->shader);
      shader.set_uniform("color_mode", 0);
      shader.set_uniform("model_matrix", all_clouds[i].second.cast<float>().matrix());
      cloud_buffer[i]->draw(shader);

      shader.set_uniform("color_mode", 1);
      shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
      shader.set_uniform("apply_keyframe_scale", true);
      auto &sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
      sphere.draw(shader);
      shader.set_uniform("apply_keyframe_scale", false);
    }
  }
  virtual void draw_gl() override
  {
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    main_canvas->bind();
    main_canvas->shader->set_uniform("color_mode", 2);
    main_canvas->shader->set_uniform("model_matrix", (Eigen::UniformScaling<float>(3.0f) * Eigen::Isometry3f::Identity()).matrix());

    const auto &coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
    coord.draw(*main_canvas->shader);

    main_canvas->shader->set_uniform("color_mode", 1);
    main_canvas->shader->set_uniform("model_matrix", (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
    main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
    const auto &grid = glk::Primitives::instance()->primitive(glk::Primitives::GRID);
    grid.draw(*main_canvas->shader);

    main_canvas->shader->set_uniform("point_scale", 1.0f);

    if (odometry_set)
      draw_clouds();

    main_canvas->unbind();
    main_canvas->render_to_screen();

    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  }

  /**
   * @brief frame buffer size change callback
   * @param size  frame buffer size
   */
  virtual void framebuffer_size_callback(const Eigen::Vector2i &size) override
  {
    main_canvas->set_size(size);
    framebuffer_size = size;
  }

private:
  /**
   * @brief draw main menu
   *
   */
  void main_menu()
  {
    ImGui::BeginMainMenuBar();

    // bool open_rosbag_dialog = false;
    // bool open_odom_directory_dialog = false;
    bool load_config_dialog = false;
    bool save_interactive_dialog = false;
    bool save_stations_dialog = false;
    if (ImGui::BeginMenu("File"))
    {
      // if (ImGui::BeginMenu("Open"))
      // {
      //   if (ImGui::MenuItem("rosbag"))
      //   {
      //     open_rosbag_dialog = true;
      //   }
      //   if (ImGui::MenuItem("odometry directory"))
      //   {
      //     open_odom_directory_dialog = true;
      //   }

      //   ImGui::EndMenu();
      // }

      // if (ImGui::BeginMenu("Load"))
      // {
      //   if (ImGui::MenuItem("config"))
      //   {
      //     load_config_dialog = true;
      //   }

      //   ImGui::EndMenu();
      // }

      if (ImGui::BeginMenu("Load"))
      {
        if (ImGui::MenuItem("config"))
        {
          pfd::open_file dialog("choose config file");
          while (!dialog.ready())
          {
            usleep(100);
          }

          if (dialog.result().empty())
          {
            return;
          }

          std::vector<std::string> config_paths = dialog.result();

          if (config_paths.size() != 1)
          {
            std::cout << "choose one and only one config!" << std::endl;
            return;
          }

          std::string config_path = config_paths[0];
          odometry_set = std::make_shared<odometry2graph>();
          odometry_set->load_config(config_path);
          load_config_dialog = true;
        }

        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Process"))
      {
        if (odometry_set)
        {
          odometry_set->process();
          is_update = true;
        }
      }

      if (ImGui::BeginMenu("Save"))
      {
        if (ImGui::MenuItem("interactive slam"))
        {
          save_interactive_dialog = true;
        }
        if (ImGui::MenuItem("stations"))
        {
          save_stations_dialog = true;
        }

        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Quit"))
      {
        close();
      }

      ImGui::EndMenu();
    }
    // open_rosbag(open_rosbag_dialog);
    // open_odom_directory(open_odom_directory_dialog);

    // open_config(load_config_dialog);

    save_interactive_directory(save_interactive_dialog);
    save_stations_directory(save_stations_dialog);

    if (ImGui::BeginMenu("View"))
    {
      if (ImGui::MenuItem("Reset camera"))
      {
        main_canvas->reset_camera();
      }
      if (ImGui::MenuItem("Projection setting"))
      {
        main_canvas->show_projection_setting();
      }
      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

private:
  // void open_config(bool load_config_dialog)
  // {

  //   if (progress->run("open_config"))
  //   {
  //     auto result = progress->result<bool>();
  //     if (!result)
  //     {
  //       pfd::message message("Error", "failed to open config", pfd::choice::ok);
  //       while (!message.ready())
  //       {
  //         usleep(100);
  //       }
  //       return;
  //     }
  //   }

  //   if (!load_config_dialog)
  //   {
  //     return;
  //   }

  //   pfd::open_file dialog("choose config file");
  //   while (!dialog.ready())
  //   {
  //     usleep(100);
  //   }

  //   if (dialog.result().empty())
  //   {
  //     return;
  //   }

  //   std::vector<std::string> config_paths = dialog.result();

  //   if (config_paths.size() != 1)
  //   {
  //     std::cout << "choose one and only one config!" << std::endl;
  //     return;
  //   }

  //   std::string config_path = config_paths[0];
  //   odometry_set.reset();
  //   odometry_set = std::make_shared<odometry2graph>();
  //   auto load_config_task = [this, config_path](guik::ProgressInterface &p)
  //   { return this->odometry_set->load_config(config_path); };
  //   progress->open<bool>("open_config", load_config_task);

  //   is_update = true;
  // }

  // void open_rosbag(bool open_rosbag_dialog)
  // {
  //   if (!open_rosbag_dialog)
  //   {
  //     return;
  //   }

  //   pfd::open_file dialog("choose rosbag");
  //   while (!dialog.ready())
  //   {
  //     usleep(100);
  //   }

  //   std::vector<std::string> rosbag_paths = dialog.result();

  //   if (rosbag_paths.size() != 1)
  //   {
  //     std::cout << "choose only one rosbag!" << std::endl;
  //     return;
  //   }

  //   rosbag_path = rosbag_paths[0];

  //   return;
  // }
  // bool load_rosbag_odom_directory(guik::ProgressInterface &progress, const std::string &rosbag_path, const std::string &directory)
  // {
  //   progress.set_title("Opening " + directory);
  //   progress.set_text("loading odometry");

  //   // progress.increment();

  //   odometry_set = std::make_shared<odometry2graph>(rosbag_path, directory,
  //                                                   downsample_resolution, is_downsample,
  //                                                   keyframe_delta_x, keyframe_delta_angle,
  //                                                   friends_num);

  //   return true;
  // }

  // bool load_odom_directory(guik::ProgressInterface &progress, const std::string &directory)
  // {
  //   progress.set_title("Opening " + directory);
  //   progress.set_text("loading odometry");

  //   // progress.increment();

  //   odometry_set = std::make_shared<odometry2graph>(directory,
  //                                                   downsample_resolution, is_downsample,
  //                                                   keyframe_delta_x, keyframe_delta_angle,
  //                                                   friends_num);

  //   return true;
  // }

  // void open_odom_directory(bool open_odom_directory_dialog)
  // {
  //   if (progress->run("open_odom_directory"))
  //   {
  //     auto result = progress->result<bool>();
  //     if (!result)
  //     {
  //       pfd::message message("Error", "failed to open odometry data", pfd::choice::ok);
  //       while (!message.ready())
  //       {
  //         usleep(100);
  //       }
  //       return;
  //     }
  //   }

  //   if (!open_odom_directory_dialog)
  //   {
  //     return;
  //   }

  //   pfd::select_folder dialog("choose odometry directory");
  //   while (!dialog.ready())
  //   {
  //     usleep(100);
  //   }

  //   if (dialog.result().empty())
  //   {
  //     return;
  //   }

  //   std::string directory = dialog.result();
  //   if (rosbag_path.empty())
  //   {
  //     auto load_odom_directory_task = [this, directory](guik::ProgressInterface &p)
  //     { return this->load_odom_directory(p, directory); };
  //     progress->open<bool>("open_odom_directory", load_odom_directory_task);
  //   }
  //   else
  //   {
  //     auto load_rosbag_odom_directory_task = [this, directory](guik::ProgressInterface &p)
  //     { return this->load_rosbag_odom_directory(p, this->rosbag_path, directory); };
  //     progress->open<bool>("open_odom_directory", load_rosbag_odom_directory_task);
  //   }
  //   is_update = true;
  // }

  bool save_interactive(guik::ProgressInterface &progress, const std::string &directory)
  {
    progress.set_title("Opening " + directory);
    progress.set_text("saveing interactive");

    // odometry_set->save_map(directory, odometry2graph::Format::INTERACTIVE_SLAM_MODE);
    odometry_set->save_map(odometry2graph::Format::INTERACTIVE_SLAM_MODE, directory);

    return true;
  }

  void save_interactive_directory(bool save_dialog)
  {
    if (progress->run("save_interactive"))
    {
      auto result = progress->result<bool>();
      if (!result)
      {
        pfd::message message("Error", "failed to save_interactive", pfd::choice::ok);
        while (!message.ready())
        {
          usleep(100);
        }
        return;
      }
    }

    if (!save_dialog)
    {
      return;
    }

    pfd::select_folder dialog("choose directory");
    while (!dialog.ready())
    {
      usleep(100);
    }

    if (dialog.result().empty())
    {
      return;
    }

    std::string directory = dialog.result();
    auto save_task = [this, directory](guik::ProgressInterface &p)
    { return this->save_interactive(p, directory); };
    progress->open<bool>("save_interactive", save_task);
  }

  bool save_stations(guik::ProgressInterface &progress, const std::string &directory)
  {
    progress.set_title("Opening " + directory);
    progress.set_text("saveing stations");

    // odometry_set->save_map(directory, odometry2graph::Format::STATION_MODE);
    odometry_set->save_map(odometry2graph::Format::STATION_MODE, directory);

    return true;
  }

  void save_stations_directory(bool save_dialog)
  {
    if (progress->run("save_stations"))
    {
      auto result = progress->result<bool>();
      if (!result)
      {
        pfd::message message("Error", "failed to save_stations", pfd::choice::ok);
        while (!message.ready())
        {
          usleep(100);
        }
        return;
      }
    }

    if (!save_dialog)
    {
      return;
    }

    pfd::select_folder dialog("choose directory");
    while (!dialog.ready())
    {
      usleep(100);
    }

    if (dialog.result().empty())
    {
      return;
    }

    std::string directory = dialog.result();
    auto save_task = [this, directory](guik::ProgressInterface &p)
    { return this->save_stations(p, directory); };
    progress->open<bool>("save_stations", save_task);
  }

  /**
   * @brief mouse event handler
   *
   */
  void mouse_control()
  {
    ImGuiIO &io = ImGui::GetIO();
    if (!io.WantCaptureMouse)
    {
      main_canvas->mouse_control();
    }
  }

private:
  Eigen::Vector2i framebuffer_size;

  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ProgressModal> progress;

  bool is_update = false;
  float keyframe_delta_x;
  float keyframe_delta_angle;
  float downsample_resolution;
  bool is_downsample;
  int friends_num;

  float last_keyframe_delta_x;
  float last_keyframe_delta_angle;
  float last_downsample_resolution;
  bool last_is_downsample;
  int last_friends_num;
  // std::shared_ptr<OdometrySet> odometry_set;
  std::string rosbag_path;

  std::shared_ptr<odometry2graph> odometry_set;

  std::vector<std::unique_ptr<glk::PointCloudBuffer>> cloud_buffer;

  std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Isometry3d>> all_clouds;
};

/**
 * @brief main
 */
int main(int argc, char **argv)
{
  std::unique_ptr<guik::Application> app(new Odometry2GraphApplication());
  data_directory = std::string(argv[1]);
  if (!app->init("Odometry2Graph", Eigen::Vector2i(1920, 1080)))
  {
    return 1;
  }

  app->run();

  return 0;
}
