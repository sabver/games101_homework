#ifndef SPRITE_H
#define SPRITE_H

#include <cstdlib>

struct Sprite
{
    float x, y;
    size_t tex_id;
    float player_dist;
    bool operator<(const Sprite &s) const
    {
        return player_dist > s.player_dist;
    }
};

#endif // SPRITE_H
