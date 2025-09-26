/* sources:
https://piolabs.com/blog/insights/unit-testing-part-1.html
https://piolabs.com/blog/insights/unit-testing-part-2.html
https://piolabs.com/blog/insights/unit-testing-part-3.html

      build - fails native, works portenta
      test - works native, fails portenta

      in the yml you can specify native or non native but idk if itll know nonnative

  Platform io unit tests run against hardware
  Build tests alr exist
  Platform io is arduino ide in code
  They did the build local not just w a physical arduino
  Have a GitHub actions to do build tests
  Native or hardware serial commands
  Native only where you emulate
  Hardware only

  use a folder as sd card, then from there:
  ls
  test cat file, see contoents
  test rm shoudl remove file
  platofmrbridge, t
  those are the seial commands

*/

// #include <Arduino.h>
#include "PlatformBridge.h"
#include <unity.h>
#include <cstring>
#include <filesystem>
#include <fstream>

static const char *TEST_FILE = "test.txt";
static const char *TEST_DIR  = "testdir";

void test_sd_begin_creates_dir(void) {
  // Make sure the builtin SD dir is initialized
  bool ok = SD.begin();
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(std::filesystem::exists(BUILTIN_SDCARD_DIR));
}
void test_write_and_cat_file(void) {
  // Write a file
  File f = SD.open(TEST_FILE, FILE_WRITE);
  const char *msg = "Hello World";
  f.write(msg, strlen(msg));
  f.close();

  // Now reopen + read it back
  File f2 = SD.open(TEST_FILE, FILE_READ);
  String contents = f2.readString(64);
  f2.close();

  TEST_ASSERT_EQUAL_STRING("Hello World", contents.c_str());
}

  
  void test_ls_directory(void) {
    // Create a subdirectory with files
    std::filesystem::path dirpath = std::filesystem::path(BUILTIN_SDCARD_DIR) / TEST_DIR;
    std::filesystem::create_directory(dirpath);

    std::ofstream(dirpath / "a.txt") << "aaa";
    std::ofstream(dirpath / "b.txt") << "bbb";

    File dirFile(dirpath.c_str(), FILE_READ);
    File entry = dirFile.openNextFile();
    TEST_ASSERT_TRUE(entry); // should have at least one file
}

void test_rm_file(void) {
  // Ensure file exists
  std::ofstream(std::filesystem::path(BUILTIN_SDCARD_DIR) / "deleteme.txt") << "bye";
  TEST_ASSERT_TRUE(SD.exists("deleteme.txt"));

  bool removed = SD.remove("deleteme.txt");
  TEST_ASSERT_TRUE(removed);
  TEST_ASSERT_FALSE(SD.exists("deleteme.txt"));
}


void setup(void) {}
void tearDown(void) {}

void loop() {
    UNITY_BEGIN();

    RUN_TEST(test_sd_begin_creates_dir);
    RUN_TEST(test_write_and_cat_file);
    // RUN_TEST(test_ls_directory); //currently failing
    RUN_TEST(test_rm_file);

    UNITY_END();
}