#include <SDL2/SDL.h>
#include <Eigen/Eigen>
#include <bits/stdc++.h>

using namespace Eigen;
using Edge = std::pair<Vector4f, Vector4f>;

class Display;
class Viewport;

class Display {
    public:
        int w, h;
        Display(const char* title, int w, int h);
        ~Display();
        int drawLine(Vector2i p, Vector2i q, Vector3i color);
        int drawPixel(Vector2i p, Vector3i color);
        int fillDisplay(Vector3i color);
        Vector2i convertPointToDisplayCoordinates(Vector2f cartesian_pixel);
        Vector2i convertPointToPixel(Viewport *v, Vector4f homogeneous_point);
    private:
        SDL_Renderer *renderer;
        SDL_Window *window;
};

class Viewport {
    public:
        float fov_y, fov_x, d, f;
        Viewport(float fov_y, float fov_x, float d) {
            this->fov_x = fov_x;
            this->fov_y = fov_y;
            this->d = d;
        }
        float getNearPlaneDimensionWidth() {
            float radians = this->fov_x * M_PI / 180;
            return tan(radians) * this->d * 2;
        }
        float getNearPlaneDimensionHeight() {
            float radians = this->fov_y * M_PI / 180;
            return tan(radians) * this->d * 2;
        }
        Matrix4f perspectiveMatrix();
};

class Game {
    public:
        Game(const char *title, int w, int h){
            this->app = new Display(title, w, h);
            this->view = new Viewport(45, 45, 1);
        };
        ~Game(){
            this->app->~Display();
            free(view);
        }
        void run();
        int fillDisplay(Vector3i color){
            return app->fillDisplay(color);
        }
        void drawEdges(std::vector<Edge> edges, Vector3i color);
    private:
        Display *app;
        Viewport *view;
};

void initGame();
std::vector<Edge> buildCube(Vector3f center, float side);
void applyMatrixToVertices(Matrix4f m, std::vector<Edge> &edges);
Matrix4f buildRotationMatrix(float theta_x, float theta_y, float theta_z);
