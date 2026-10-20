#include <string>

#include "dryframe.h"

int main() {
    CHECK(1 + 1 == 2, "basic arithmetic holds");
    CHECK_EQ(2 + 2, 4);
    CHECK_EQ(std::string("a") + std::string("b"), std::string("ab"));
    CHECK_NEAR(0.1 + 0.2, 0.3, 1e-9);
    return DF_SUMMARIZE("example_test");
}