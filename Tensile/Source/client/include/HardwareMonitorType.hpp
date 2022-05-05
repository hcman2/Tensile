#pragma once
namespace Tensile
{
    namespace Client
    {
        typedef enum {
            CLK_TYPE_SYS=0x0,
            CLK_TYPE_FIRST=CLK_TYPE_SYS,
            CLK_TYPE_DF,
            CLK_TYPE_DCEF,
            CLK_TYPE_SOC,
            CLK_TYPE_MEM,
            CLK_TYPE_LAST=CLK_TYPE_MEM,
            CLK_INVALID=0xFFFFFFFF
        } ClockType;
    }
}
