#include <cabin_nav/input/image_input_data.h>

namespace cabin {

std::ostream& ImageInputData::write(std::ostream& out) const
{
    out << "<Input type: " << getType() << ", img_data: " << img_data << ">";
    return out;
}

bool ImageInputData::getImage(
        const InputData::Map& input_data_map,
        const std::string& input_name,
        ImgData& _img_data)
{
    if ( !InputData::isValid(input_data_map, input_name, "image") )
    {
        return false;
    }

    ImageInputData::ConstPtr image_input_data =
        std::static_pointer_cast<ImageInputData>(
                input_data_map.at(input_name));

    _img_data = image_input_data->img_data;

    return true;
}

} // namespace cabin
