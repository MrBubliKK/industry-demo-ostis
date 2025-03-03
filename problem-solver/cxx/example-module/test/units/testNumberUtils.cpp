/*
 * This source file is part of an OSTIS project. For the latest info, see http://ostis.net
 * Distributed under the MIT License
 * (See accompanying file COPYING.MIT or copy at http://opensource.org/licenses/MIT)
 */

#include <sc-memory/test/sc_test.hpp>
#include <sc-builder/scs_loader.hpp>

#include "utils/NumberUtils.hpp"

#include "utils/TestUtils.hpp"

namespace numberUtilsTest
{
ScsLoader loader;
std::string const EXAMPLE_MODULE_TEST_FILES_DIR_PATH = "../test-structures/";

using NumberUtilsTest = ScMemoryTest;

TEST_F(NumberUtilsTest, isPositiveIntegerTest)
{
  std::string const & positiveNumberStr = "250";
  ASSERT_TRUE(utils::NumberUtils::isPositiveInteger(positiveNumberStr));
  std::string const & notANumberStr = "number_250";
  ASSERT_FALSE(utils::NumberUtils::isPositiveInteger(notANumberStr));
  std::string const & notAPositiveNumberStr = "-250";
  ASSERT_FALSE(utils::NumberUtils::isPositiveInteger(notAPositiveNumberStr));
}

TEST_F(NumberUtilsTest, resolveEistingNumberTest)
{
  ScAgentContext & context = *m_ctx;

  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "number.scs");

  ScAddr existingNumber = context.SearchElementBySystemIdentifier("number_100");
  ASSERT_TRUE(existingNumber.IsValid());

  ScAddr const & resolvedNumber = utils::NumberUtils::resolveNumber(context, 100);
  ASSERT_EQ(resolvedNumber, existingNumber);
}

TEST_F(NumberUtilsTest, resolveNonExistingNumberTest)
{
  ScAgentContext & context = *m_ctx;

  ScAddr const & resolvedNumber = utils::NumberUtils::resolveNumber(context, 100);

  std::string lengthNumberIdtf;
  utils::TestUtils::getSoleIdtf(context, resolvedNumber, lengthNumberIdtf);
  ASSERT_EQ("100", lengthNumberIdtf);
}

}  // namespace numberUtilsTest
