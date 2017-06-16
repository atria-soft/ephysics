/** @file
 * @author Edouard DUPIN
 * @copyright 2011, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <elog/log.hpp>

namespace ephysic {
	int32_t getLogId();
};
#define EPHY_BASE(info,data) ELOG_BASE(ephysic::getLogId(),info,data)

#define EPHY_PRINT(data)         EPHY_BASE(-1, data)
#define EPHY_CRITICAL(data)      EPHY_BASE(1, data)
#define EPHY_ERROR(data)         EPHY_BASE(2, data)
#define EPHY_WARNING(data)       EPHY_BASE(3, data)
#define EPHY_INFO(data)          EPHY_BASE(4, data)
#ifdef DEBUG
	#define EPHY_DEBUG(data)         EPHY_BASE(5, data)
	#define EPHY_VERBOSE(data)       EPHY_BASE(6, data)
	#define EPHY_TODO(data)          EPHY_BASE(4, "TODO : " << data)
#else
	#define EPHY_DEBUG(data)         do { } while(false)
	#define EPHY_VERBOSE(data)       do { } while(false)
	#define EPHY_TODO(data)          do { } while(false)
#endif

#define EPHY_ASSERT(cond,data) \
	do { \
		if (!(cond)) { \
			EPHY_CRITICAL(data); \
			assert(!#cond); \
		} \
	} while (0)

