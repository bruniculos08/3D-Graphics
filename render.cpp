#include "render.h"

int main(){
    initGame();
    return 0;
}

Display::Display(const char *title, int w, int h){
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;

    if (SDL_Init(SDL_INIT_EVERYTHING ^ SDL_INIT_JOYSTICK) >= 0){
        window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        w, h, SDL_WINDOW_SHOWN);
        if (window)
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    else{
        std::cout << "Could not create window." << std::endl;
        exit(1);
    }
    
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    this->w = w;
    this->h = h;
    this->window = window;
    this->renderer = renderer;
}

Display::~Display(){
    SDL_Quit();
}

int Display::drawPixel(Vector2i pixel, Vector3i color){
    SDL_SetRenderDrawColor(this->renderer, color(0), color(1), color(2), 255);
    SDL_RenderDrawPoint(this->renderer, pixel(0), pixel(1));
    SDL_RenderPresent(this->renderer);
    return 0;
}

int Display::drawLine(Vector2i p, Vector2i q, Vector3i color){
    SDL_SetRenderDrawColor(this->renderer, color(0), color(1), color(2), 255);
    SDL_RenderDrawLine(this->renderer, p(0), p(1), q(0), q(1));
    SDL_RenderPresent(this->renderer);
    return 0;
}

int Display::fillDisplay(Vector3i color){
    SDL_SetRenderDrawColor(this->renderer, color(0), color(0), color(0), 255);
    SDL_RenderClear(this->renderer);
    return 0;
}

Vector2i Display::convertPointToDisplayCoordinates(Vector2f cartesian_pixel){
    // Convert a point normalized such as x,y between 1 and -1:
    int x = (int) (this->w/2) + (cartesian_pixel(0)) * (this->w/2);
    int y = (int) (this->h/2) - (cartesian_pixel(1)) * (this->h/2);
    Vector2i screen_pixel(x,y);
    return screen_pixel;
}

Vector2i Display::convertPointToPixel(Viewport *v, Vector4f homogenous_point){
    // Convert a point in the frustrum space to a normalized point (in a box) such as x,y between 1 and -1:
    Vector4f projected_point = v->perspectiveMatrix() * homogenous_point;
    Vector2f pixel(projected_point(0)/projected_point(3), projected_point(1)/projected_point(3));
    return this->convertPointToDisplayCoordinates(pixel);
}

Matrix4f Viewport::perspectiveMatrix(){
    Matrix4f perspective_matrix = Matrix4f::Zero();
    // Multiply (x, y) by "d" because:
    //  x' = (x * d)/z
    //  y' = (y * d)/z
    // where x' and y' are the point coordinates in the near plane.
    perspective_matrix(0,0) = this->d;
    perspective_matrix(1,1) = this->d; 
    perspective_matrix(2,2) = 1;
    perspective_matrix(3,2) = 1;
    // Now, to put every point inside the frustrum into a box with vertices
    // (-1, -1, 0), (1, -1, 0), (-1, 1, 0), (1, 1, 0),
    // (-1, -1, 1), (1, -1, 1), (-1, 1, 1), (1, 1, 1)
    // let's first put z into the interval [0, 1], to do that
    // we must solve the following equation:
    //      (a * d + b)/d = 0
    //      (a * f + b)/f = 1
    //      a * (f - d) = f 
    //      a = f / (f - d) 
    //      b = - (f * d)/(f - d)
    // then we must set:
    perspective_matrix(2,2) *= (this->f)/(this->f - this->d);  
    perspective_matrix(2,3) = -(this->f * this->d)/(this->f - this->d);
    // To set x and y into the interval [-1, 1] we must divide it by the frustrum dimensions:
    perspective_matrix(0,0) *= 2/(this->getNearPlaneDimensionWidth());
    perspective_matrix(1,1) *= 2/(this->getNearPlaneDimensionHeight());
    return perspective_matrix;
}

void Game::drawEdges(std::vector<std::pair<Vector4f, Vector4f>> edges, Vector3i color){
    for(std::pair<Vector4f, Vector4f> e : edges){
        Vector4f v1 = e.first;
        Vector4f v2 = e.second;
        Vector2i p1 = this->app->convertPointToPixel(this->view, v1);
        Vector2i p2 = this->app->convertPointToPixel(this->view, v2);
        this->app->drawLine(p1, p2, color);
    }
}

std::vector<Edge> buildCube(Vector3f center, float side){
    float max_x = center(0) + side/2;
    float min_x = center(0) - side/2;
    float max_y = center(1) + side/2;
    float min_y = center(1) - side/2;
    float max_z = center(2) + side/2;
    float min_z = center(2) - side/2;
    std::vector<Edge> cube = {
        std::make_pair(Vector4f(max_x, max_y, min_z, 1), Vector4f(max_x, max_y, max_z, 1)),
        std::make_pair(Vector4f(min_x, max_y, min_z, 1), Vector4f(min_x, max_y, max_z, 1)),

        std::make_pair(Vector4f(max_x, min_y, min_z, 1), Vector4f(max_x, min_y, max_z, 1)),
        std::make_pair(Vector4f(min_x, min_y, min_z, 1), Vector4f(min_x, min_y, max_z, 1)),
        
        std::make_pair(Vector4f(max_x, max_y, min_z, 1), Vector4f(min_x, max_y, min_z, 1)),
        std::make_pair(Vector4f(max_x, min_y, min_z, 1), Vector4f(min_x, min_y, min_z, 1)),
        std::make_pair(Vector4f(max_x, min_y, min_z, 1), Vector4f(max_x, max_y, min_z, 1)),
        std::make_pair(Vector4f(min_x, min_y, min_z, 1), Vector4f(min_x, max_y, min_z, 1)),
        
        std::make_pair(Vector4f(max_x, max_y, max_z, 1), Vector4f(min_x, max_y, max_z, 1)),
        std::make_pair(Vector4f(max_x, min_y, max_z, 1), Vector4f(min_x, min_y, max_z, 1)),
        std::make_pair(Vector4f(max_x, min_y, max_z, 1), Vector4f(max_x, max_y, max_z, 1)),
        std::make_pair(Vector4f(min_x, min_y, max_z, 1), Vector4f(min_x, max_y, max_z, 1)),
    };
    return cube;
}

void applyMatrixToVertices(Matrix4f m, std::vector<Edge> &edges){
    for(std::pair<Vector4f, Vector4f> &e : edges){
        e.first = m * e.first; 
        e.second = m * e.second; 
    }
}

Matrix4f buildRotationMatrix(float angle_x, float angle_y, float angle_z){
    Matrix4f m = Matrix4f::Identity();
    float radians_x = (angle_x / 180) * M_PI;
    float radians_y = (angle_y / 180) * M_PI;
    float radians_z = (angle_z / 180) * M_PI;
    
    if (angle_x != 0.0f) {
        Matrix4f r_x = Matrix4f::Identity();
        r_x(1,1) = cos(radians_x);
        r_x(1,2) = -sin(radians_x);
        r_x(2,1) = sin(radians_x);
        r_x(2,2) = cos(radians_x);
        m = r_x;
    }
    
    if (angle_y != 0.0f) {
        Matrix4f r_y = Matrix4f::Identity();
        r_y(0,0) = cos(radians_y);
        r_y(0,2) = sin(radians_y);
        r_y(2,0) = -sin(radians_y);
        r_y(2,2) = cos(radians_y);
        m *= r_y;
    }
    
    if (angle_z != 0.0f) {
        Matrix4f r_z = Matrix4f::Identity();
        r_z(0,0) = cos(radians_z);
        r_z(1,0) = sin(radians_z);
        r_z(0,1) = -sin(radians_z);
        r_z(1,1) = cos(radians_z);
        m *= r_z;
    }
    
    return m;
}

Matrix4f buildTranslationMatrix(float delta_x, float delta_y, float delta_z){
    Matrix4f m = Matrix4f::Identity();
    m(0, 3) = delta_x;
    m(1, 3) = delta_y;
    m(2, 3) = delta_z;
    return m;
}

void initGame(){
    Game game("teste", 640, 480);
    auto cube = buildCube(Vector3f(0.0f, 0.0f, 2.5f), 1.0);
    for(int i = 0; i < 200; i++){
        auto first_translation = buildTranslationMatrix(0.0f, 0.0f, -2.5f);
        auto rotation = buildRotationMatrix(1.0f, 1.0f, 1.0f);
        auto second_translation = buildTranslationMatrix(0.0f, 0.0f, 2.5f);
        applyMatrixToVertices(second_translation * rotation * first_translation, cube);
        game.fillDisplay(Vector3i(0, 0, 0));
        game.drawEdges(cube, Vector3i(0, 255, 0));
        SDL_Delay(10);
    }
    game.~Game();
}