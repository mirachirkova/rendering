#include "common.h"
#include "camera.h"
#include "scene.h"
#include "object.h"
#include "mesh.h"

#include "LiteMath/LiteMath.h"
#include "LiteMath/Image2d.h"

// stb_image is a single-header C library, which means one of your cpp files must have
//    #define STB_IMAGE_IMPLEMENTATION
//    #define STB_IMAGE_WRITE_IMPLEMENTATION
// since Image2d already defines the implementation, we don't need to do that here.
#include "stb_image.h"
#include "stb_image_write.h"

#include <SDL_keycode.h>
#include <SDL.h>dex.ru

#include <cstdint>
#include <iostream>
#include <fstream>
using namespace cmesh4;


// You must include the command line parameters for your main function to be recognized by SDL
int main(int argc, char **args)
{
  const int SCREEN_WIDTH = 960;
  const int SCREEN_HEIGHT = 960;
  const LiteMath::float3 CAMERA_POSITION = {1.0f, 2.00f, -2.00f};
  const LiteMath::float3 LIGHT_POSITION = {1.0f, 1.0f, 1.0f};
  const float ZOOM_COEFF = 1.0f;
  const bool SHADOWS = false;
  
  std::vector<std::unique_ptr<IObject>> objects;
  objects.push_back(create_object_plane(0,1,0,0, Colour{113, 179,60,0}, false));
  //objects.push_back(create_object_plane(0,1,0,0, Colour{113, 179,60,0}, true));
  //objects.push_back(create_object_plane(1,1,1,1, {0, 255,0,0}));
  //objects.push_back(create_object_mesh("SampleObjects/cube.obj"));
  objects.push_back(create_object_mesh("SampleObjects/spot.obj", Colour{203,192,255,0}));
  //objects.push_back(create_object_mesh("SampleObjects/stanford-bunny.obj"));
  //objects.push_back(create_object_mesh("SampleObjects/MotorcycleCylinderHead.obj"));
  //objects.push_back(create_object_mesh("SampleObjects/as1-oc-214.obj"));
  /*objects.push_back(create_object_triangle(
    LiteMath::float3{27,	9, 64},
    LiteMath::float3{45,	70, 41},
    LiteMath::float3{79,	35,	78},
    LiteMath::float3{0, 1, 0},
    LiteMath::float3{0, 1, 0},
    LiteMath::float3{0, 1, 0}));*/
  std::unique_ptr<IScene> scene = create_scene(std::move(objects), LIGHT_POSITION);


  
  std::unique_ptr<ICamera> camera = create_camera(std::move(scene), SCREEN_WIDTH, SCREEN_HEIGHT,
    CAMERA_POSITION);
  camera->update_zoom(ZOOM_COEFF);
  Picture pixels (SCREEN_HEIGHT*SCREEN_WIDTH);

  // Initialize SDL. SDL_Init will return -1 if it fails.
  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
  {
    std::cerr << "Error initializing SDL: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create our window
  SDL_Window *window = SDL_CreateWindow("SDF Viewer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

  // Make sure creating the window succeeded
  if (!window)
  {
    std::cerr << "Error creating window: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create a renderer
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  if (!renderer)
  {
    std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  // Create a texture
  SDL_Texture *texture = SDL_CreateTexture(
      renderer,
      SDL_PIXELFORMAT_ARGB8888,    // 32-bit RGBA format
      SDL_TEXTUREACCESS_STREAMING, // Allows us to update the texture
      SCREEN_WIDTH,
      SCREEN_HEIGHT);

  if (!texture)
  {
    std::cerr << "Texture could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  SDL_Event ev;
  bool running = true;

  // Main loop
  while (running)
  {
    // Event loop
    while (SDL_PollEvent(&ev) != 0)
    {
      // check event type
      switch (ev.type)
      {
      case SDL_QUIT:
        // shut down
        running = false;
        break;
      case SDL_KEYDOWN:
        // test keycode
        switch (ev.key.keysym.sym)
        {
        //W and S keys to change the slice of the grid currently rendered
        case SDLK_w:
          camera->move_forward();
          break;
        case SDLK_s:
          camera->move_backward();
          break;
        case SDLK_a:
          camera->move_left();
          break;
        case SDLK_d:
          camera->move_right();
          break;
        case SDLK_q:
          camera->move_down();
          break;
        case SDLK_e:
          camera->move_up();
          break;
        case SDLK_p:
          camera->zoom_in();
          break;
        case SDLK_m:
          camera->zoom_out();
          break;
        //ESC to exit 
        case SDLK_ESCAPE:
          running = false;
          break;
          // etc
        }
        break;
      }
    }

    // Update pixel buffer
    camera->take_picture(pixels, SHADOWS);

    // Update the texture with the pixel buffer
    SDL_UpdateTexture(texture, nullptr, pixels.data(), SCREEN_WIDTH * sizeof(uint32_t));

    // Clear the renderer
    SDL_RenderClear(renderer);

    // Copy the texture to the renderer
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);

    // Update the screen
    SDL_RenderPresent(renderer);
  }

  // Destroy the window. This will also destroy the surface
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);

  // Quit SDL
  SDL_Quit();

  // End the program
  return 0;
}
