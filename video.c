#include "video.h"
#include <SDL.h>
#include<SDL_image.h>
#include "vec2.h"
#include "drone.h"

SDL_Texture* drtxt;

Display* initVideo(int width,int height){
    SDL_Init(SDL_INIT_EVERYTHING);

    Display* d = malloc(sizeof(Display));
    d->width = width;
    d->height = height;
    d->wnd = SDL_CreateWindow("Vocabs",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,width,height,SDL_WINDOW_SHOWN);
    d->rnd = SDL_CreateRenderer(d->wnd,-1,SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    IMG_Init(IMG_INIT_PNG);
    SDL_Surface* srf = IMG_Load("drone.png");
    drtxt = SDL_CreateTextureFromSurface(d->rnd,srf);
    SDL_FreeSurface(srf);
    return d;
}
void quitVideo(Display* d){
    SDL_DestroyRenderer(d->rnd);
    SDL_DestroyWindow(d->wnd);
    free(d);
    SDL_Quit();
}
void clear(Display* disp){
    SDL_SetRenderDrawColor(disp->rnd,32,64,128,255);
    SDL_RenderClear(disp->rnd);
}
void drawDrone(Display*d, Drone* dr){
    SDL_Rect rct;
    rct.x = DROUND(dr->position.x-0.5*dr->size);
    rct.y = DROUND(dr->position.y-0.5*dr->size);
    rct.h = dr->size;
    rct.w = dr->size;
    SDL_RenderCopy(d->rnd,drtxt,NULL,&rct);
}

void render(Display* disp){
    SDL_RenderPresent(disp->rnd);
}