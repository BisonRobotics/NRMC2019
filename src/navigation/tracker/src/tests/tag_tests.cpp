#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <tracker/tag.h>


using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

using namespace tracker;

TEST(TagTests, initialization)
{
  EXPECT_EQ(Tag::getTags().size(), 0);

  Tag one(1, 0.5);
  EXPECT_EQ(one.getID(), 1);
  EXPECT_EQ(one.getSeq(), 0);
  EXPECT_EQ(one.getTransformsSize(), 0);
  EXPECT_EQ(one.getSize(), 0.5);
  EXPECT_EQ(one.getRelativeTransforms().size(), 0);
  EXPECT_EQ(Tag::getTags().size(), 1);
  EXPECT_EQ(Tag::isInitialized(), false);
  EXPECT_EQ(Tag::getListSize(), 0);

  Tag two(2, 0.5);
  EXPECT_EQ(two.getID(), 2);
  EXPECT_EQ(two.getSeq(), 0);
  EXPECT_EQ(two.getTransformsSize(), 0);
  EXPECT_EQ(two.getSize(), 0.5);
  EXPECT_EQ(two.getRelativeTransforms().size(), 0);
  EXPECT_EQ(Tag::getTags().size(), 2);
  EXPECT_EQ(Tag::isInitialized(), false);
  EXPECT_EQ(Tag::getListSize(), 0);

  Tag two_2(2, 0.5);
  EXPECT_EQ(two_2.getID(), 2);
  EXPECT_EQ(two_2.getSeq(), 0);
  EXPECT_EQ(two_2.getTransformsSize(), 0);
  EXPECT_EQ(two_2.getSize(), 0.5);
  EXPECT_EQ(two_2.getRelativeTransforms().size(), 0);
  EXPECT_EQ(Tag::getTags().size(), 2);
  EXPECT_EQ(Tag::isInitialized(), false);
  EXPECT_EQ(Tag::getListSize(), 0);

  Tag::init(10, ros::Duration(0.5), false);
  EXPECT_EQ(Tag::isInitialized(), true);
  EXPECT_EQ(Tag::getListSize(), 10);
}

TEST(TagTests, equalityOperators)
{
  Tag one(1, 0.5);
  Tag one_duplicate(1, 0.5);
  Tag one_clone(one);
  Tag two(2, 0.5);

  EXPECT_EQ(one, one_duplicate);
  EXPECT_EQ(one, one_clone);
  EXPECT_NE(one, two);
}

TEST(TagTests, tagVector)
{
  EXPECT_EQ(Tag::getTags().size(), 0);

  Tag one(1, 0.5);
  Tag two(2, 0.5);
  Tag two_duplicate(2, 0.5);
  Tag three(3, 0.5);

  TagsVector tags = Tag::getTags();
  EXPECT_EQ(one, tags[0]);
  EXPECT_EQ(two, tags[1]);
  EXPECT_EQ(two_duplicate, tags[1]);
  EXPECT_EQ(three, tags[2]);
  EXPECT_EQ(tags.size(), 3);
}

TEST(TagTests, relativeTransform)
{
  // TODO
  EXPECT_EQ(1, 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}