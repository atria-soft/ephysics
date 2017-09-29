/** @file
 * @author Edouard DUPIN
 * @copyright 2017, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <etk/types.hpp>
#include <test-debug/debug.hpp>
#include <etk/Vector.hpp>
#include <etk/String.hpp>
#include <etk/etk.hpp>
#include <etk/stdTools.hpp>
#include <etk/String.hpp>


#include <etest/etest.hpp>

int main(int argc, const char *argv[]) {
	// init test engine:
	etest::init(argc, argv);
	return RUN_ALL_TESTS();
}
