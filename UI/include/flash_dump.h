#pragma once


// abstract:
// state variable
// button for beginning dump
//  - prompt user for output file path
//  - sends start command over fv serial
//  - dump over selected dump serial, if none is selected then dump over fv serial
// upon completion, switch to CSR window
//  - pre-fill the open file path with dump output
namespace FlashDump
{
    bool is_in_progress();

    void render();

    // separate update and render functions exposed so that the dump can continue even if dump menu is closed
    // should be called unconditionally every frame
    void update();
};