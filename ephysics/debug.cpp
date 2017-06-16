/** @file
 * @author Edouard DUPIN
 * @copyright 2011, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <ephysics/debug.hpp>

int32_t ephysic::getLogId() {
	static int32_t g_val = elog::registerInstance("ephysic");
	return g_val;
}
