#ifndef CABIN_IMAGE_INPUT_DATA_H
#define CABIN_IMAGE_INPUT_DATA_H

#include <cabin_nav/input/input_data.h>
#include <cabin_nav/structs/img_data.h>

namespace cabin {

class ImageInputData : public InputData
{
    public:

        using Ptr = std::shared_ptr<ImageInputData>;
        using ConstPtr = std::shared_ptr<const ImageInputData>;

        ImgData img_data;

        ImageInputData():
            InputData("image") {}

        virtual ~ImageInputData() = default;

        std::ostream& write(std::ostream& out) const;

        static bool getImage(
                const InputData::Map& input_data_map,
                const std::string& input_name,
                ImgData& _img_data);

};

} // namespace cabin

#endif // CABIN_IMAGE_INPUT_DATA_H
