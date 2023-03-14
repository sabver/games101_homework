#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cassert>
#include <sstream>
#include <iomanip>

#include "map.h"
#include "utils.h"
#include "player.h"
#include "sprite.h"
#include "framebuffer.h"
#include "textures.h"

int wall_x_texcoord(const float hitx, const float hity, Texture &tex_walls)
{
    float x = hitx - floor(hitx + .5); // x and y contain (signed) fractional parts of hitx and hity,
    float y = hity - floor(hity + .5); // they vary between -0.5 and +0.5, and one of them is supposed to be very close to 0
    int tex = x * tex_walls.size;
    if (std::abs(y) > std::abs(x)) // we need to determine whether we hit a "vertical" or a "horizontal" wall (w.r.t the map)
        tex = y * tex_walls.size;
    if (tex < 0) // do not forget x_texcoord can be negative, fix that
        tex += tex_walls.size;
    assert(tex >= 0 && tex < (int)tex_walls.size);
    return tex;
}

void map_show_sprite(Sprite &sprite, FrameBuffer &fb, Map &map)
{
    // map每单位的长度对应的像素
    const size_t rect_w = fb.w / (map.w * 2); // size of one map cell on the screen
    const size_t rect_h = fb.h / map.h;
    // -3 move to center when width is 6
    fb.draw_rectangle(sprite.x * rect_w - 3, sprite.y * rect_h - 3, 6, 6, pack_color(255, 0, 0));
}

void draw_sprite(Sprite &sprite, std::vector<float> &depth_buffer, FrameBuffer &fb, Player &player, Texture &tex_sprites)
{
    // absolute direction from the player to the sprite (in radians)
    float sprite_dir = atan2(sprite.y - player.y, sprite.x - player.x);
    while (sprite_dir - player.a > M_PI)
        sprite_dir -= 2 * M_PI; // remove unncesessary periods from the relative direction
    while (sprite_dir - player.a < -M_PI)
        sprite_dir += 2 * M_PI;

    float sprite_dist = std::sqrt(pow(player.x - sprite.x, 2) + pow(player.y - sprite.y, 2)); // distance from the player to the sprite
    // 利用近大远小的效果来估算sprite的宽度
    size_t sprite_screen_size = std::min(1000, static_cast<int>(fb.h / sprite_dist)); // screen sprite size
    // h_offset和v_offset给出了精灵的左上角的屏幕坐标
    // 为什么是- tex_sprites.size / 2 而不是 - sprite_screen_size / 2
    int h_offset = (sprite_dir - player.a) / player.fov * (fb.w / 2) + (fb.w / 2) / 2 - tex_sprites.size / 2; // do not forget the 3D view takes only a half of the framebuffer
    // - sprite_screen_size / 2 是为了居中
    int v_offset = fb.h / 2 - sprite_screen_size / 2;

    for (size_t i = 0; i < sprite_screen_size; i++)
    {
        if (h_offset + int(i) < 0 || h_offset + i >= fb.w / 2)
            continue;
        if (depth_buffer[h_offset + i] < sprite_dist)
            continue; // this sprite column is occluded
        for (size_t j = 0; j < sprite_screen_size; j++)
        {
            if (v_offset + int(j) < 0 || v_offset + j >= fb.h)
                continue;
            uint32_t color = tex_sprites.get(i * tex_sprites.size / sprite_screen_size, j * tex_sprites.size / sprite_screen_size, sprite.tex_id);
            uint8_t r, g, b, a;
            unpack_color(color, r, g, b, a);
            if (a > 128)
                fb.set_pixel(fb.w / 2 + h_offset + i, v_offset + j, color);
        }
    }
}

void render(FrameBuffer &fb, Map &map, Player &player, std::vector<Sprite> &sprites, Texture &tex_walls, Texture &tex_monst)
{
    fb.clear(pack_color(255, 255, 255)); // clear the screen

    const size_t rect_w = fb.w / (map.w * 2); // size of one map cell on the screen
    const size_t rect_h = fb.h / map.h;
    for (size_t j = 0; j < map.h; j++)
    { // draw the map
        for (size_t i = 0; i < map.w; i++)
        {
            if (map.is_empty(i, j))
                continue; // skip empty spaces
            size_t rect_x = i * rect_w;
            size_t rect_y = j * rect_h;
            size_t texid = map.get(i, j);
            assert(texid < tex_walls.count);
            fb.draw_rectangle(rect_x, rect_y, rect_w, rect_h, tex_walls.get(0, 0, texid)); // the color is taken from the upper left pixel of the texture #texid
        }
    }
    std::vector<float> depth_buffer(fb.w / 2, 1e3);
    for (size_t i = 0; i < fb.w / 2; i++)
    { // draw the visibility cone AND the "3D" view
        float angle = player.a - player.fov / 2 + player.fov * i / float(fb.w / 2);
        for (float t = 0; t < 20; t += .01)
        { // ray marching loop
            float x = player.x + t * cos(angle);
            float y = player.y + t * sin(angle);
            fb.set_pixel(x * rect_w, y * rect_h, pack_color(160, 160, 160)); // this draws the visibility cone

            if (map.is_empty(x, y))
                continue;

            size_t texid = map.get(x, y); // our ray touches a wall, so draw the vertical column to create an illusion of 3D
            assert(texid < tex_walls.count);
            // size_t column_height = fb.h / (t * cos(angle - player.a));
            float dist = t * cos(angle - player.a);
            depth_buffer[i] = dist;
            size_t column_height = fb.h / dist;
            int x_texcoord = wall_x_texcoord(x, y, tex_walls);
            std::vector<uint32_t> column = tex_walls.get_scaled_column(texid, x_texcoord, column_height);
            int pix_x = i + fb.w / 2; // we are drawing at the right half of the screen, thus +fb.w/2
            for (size_t j = 0; j < column_height; j++)
            { // copy the texture column to the framebuffer
                int pix_y = j + fb.h / 2 - column_height / 2;
                if (pix_y >= 0 && pix_y < (int)fb.h)
                {
                    fb.set_pixel(pix_x, pix_y, column[j]);
                }
            }
            break;
        } // ray marching loop
    }     // field of view ray sweeping
    for (size_t i = 0; i < sprites.size(); i++)
    {
        map_show_sprite(sprites[i], fb, map);
        draw_sprite(sprites[i], depth_buffer, fb, player, tex_monst);
    }
}

int main()
{
    FrameBuffer fb{1024, 512, std::vector<uint32_t>(1024 * 512, pack_color(255, 255, 255))};
    Player player{3.456, 2.345, 1.523, M_PI / 3.};
    Map map;
    Texture tex_walls("../walltext.png");
    Texture tex_monst("../monsters.png");
    if (!tex_walls.count || !tex_monst.count)
    {
        std::cerr << "Failed to load textures" << std::endl;
        return -1;
    }
    std::vector<Sprite> sprites{{3.523, 3.812, 2}, {1.834, 8.765, 0}, {5.323, 5.365, 1}, {4.123, 10.265, 1}};

    render(fb, map, player, sprites, tex_walls, tex_monst);
    drop_ppm_image("./out.ppm", fb.img, fb.w, fb.h);

    return 0;
}
