cv::Mat SubsectionWidget::Filters::detectQR(cv::Mat frame, int mode){
    std::vector<cv::Point> points;
    std::string decoded_text;
    if(mode == 0){
        cv::QRCodeDetector qr_decoder;
        try{
            decoded_text = qr_decoder.detectAndDecode(frame, points);
        }
        catch(const cv::Exception& e){
            return placeText("NO QR CODE DETECTED, EXC", frame);
        }
    }
    else{
        cv::Mat grayscale;
        cv::cvtColor(frame, grayscale, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grayscale, grayscale, cv::Size(3, 3), 0);
        grayscale = grayscale.clone();
        cv::resize(grayscale, grayscale, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        zbar::Image zbar_frame(grayscale.cols, grayscale.rows, "Y800", (uchar*)grayscale.data, grayscale.cols*grayscale.rows);
        int codes = scanner.scan(zbar_frame);

        if(codes == 0)
            return placeText("NO QR CODE DETECTED", frame);

        for(zbar::Image::SymbolIterator symbol = zbar_frame.symbol_begin(); symbol != zbar_frame.symbol_end(); ++symbol){
            decoded_text = symbol->get_data();
            for(int i = 0; i < symbol->get_location_size(); ++i) {
                points.emplace_back(cv::Point(symbol->get_location_x(i)/2.0, symbol->get_location_y(i)/2.0));
            }
        }
    }

    if(!decoded_text.empty() && !points.empty()){
        std::vector<std::vector<cv::Point>> contour = { points };
        cv::polylines(frame, contour, true, cv::Scalar(0, 0, 255), 5);
        cv::Point corner = *std::min_element(points.begin(), points.end(), [](const cv::Point& a, const cv::Point& b){
            return (a.x + a.y) < (b.x + b.y);
        });
        cv::putText(frame, decoded_text, corner+cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
    }
    else
        frame = placeText("NO QR CODE DETECTED", frame);

    return frame;
}

cv::Mat SubsectionWidget::Filters::detectShape(cv::Mat frame, int corner, bool mode, int threshold, double shape_tolerance){
    double scale = 1.0, min_dis = DBL_MAX;
    cv::Mat gray_frame, gray_resized, inv_thresh, inv_task_sector, task_sector;
    std::vector<cv::Vec3f> circles;
    std::vector<std::vector<cv::Point>> contours, shapes;
    cv::Rect sector;

    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::resize(gray_frame, gray_resized, cv::Size(), 1.0/scale, 1.0/scale, cv::INTER_AREA);
    cv::threshold(gray_resized, inv_thresh, threshold, 255, cv::THRESH_BINARY_INV);

    std::vector<cv::Point> cont;
    if(mode){
        cv::Vec3f circ_sector;
        cv::HoughCircles(gray_resized, circles, cv::HOUGH_GRADIENT, 1, gray_resized.rows/8.0, 100, 50, gray_resized.rows/8, gray_resized.rows/4);
        for(int i = 0; i < circles.size(); i++) {
            double x = circles[i][0] * scale,
                y = circles[i][1] * scale,
                r = circles[i][2] * scale;
            double dis = (x*x) + (frame.rows-y)*(frame.rows-y);
            cv::circle(frame, cv::Point(x, y), r, cv::Scalar(255, 0, 0), 4);
            if(dis < min_dis){
                min_dis = dis;
                circ_sector = cv::Vec3f(x, y, r);
            }
        }
        if(min_dis == DBL_MAX)
            return placeText("NO CIRCLES", frame);
        cv::Mat mask = cv::Mat::zeros(inv_thresh.size(), CV_8UC1);
        cv::circle(mask, cv::Point(circ_sector[0], circ_sector[1]), circ_sector[2], cv::Scalar(255), -1);
        cv::bitwise_and(inv_thresh, inv_thresh, inv_task_sector, mask);
        sector = cv::boundingRect(mask);
        inv_task_sector = inv_task_sector(sector);
    }
    else{
        cv::findContours(inv_thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2){
            return cv::contourArea(c1) > cv::contourArea(c2);
        });
        for(int i = 0; i < contours.size(); i++){
            if(cv::contourArea(contours[i]) < 1000) break;
            cv::Rect contour = cv::boundingRect(contours[i]);
            cv::Point center(contour.x + contour.width/2, contour.y + contour.height/2);
            double dis = cv::norm(center - cv::Point(((corner == 0 || corner == 2) ? 0 : inv_thresh.cols), (corner <= 1 ? 0 : inv_thresh.rows)));
            if(dis < min_dis){
                min_dis = dis;
                sector = contour;
                cont = contours[i];
            }
        }
        if(min_dis == DBL_MAX)
            return placeText("NO CONTOURS", frame);
        inv_task_sector = inv_thresh(sector);
    }
    cv::rectangle(frame, sector, cv::Scalar(0, 0, 255), 3);

    cv::bitwise_not(inv_task_sector, task_sector);
    cv::findContours(task_sector, shapes, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //frame = frame(sector);
    //cv::drawContours(frame, shapes, -1, cv::Scalar(0, 255, 0), 1);

    qDebug() << "found " << shapes.size();

    min_dis = DBL_MAX;
    std::vector<cv::Point> shape;
    cv::Point sector_center(task_sector.cols/2, task_sector.rows/2);
    for(int i = 0; i < shapes.size(); i++){
        double area = cv::contourArea(shapes[i]);
        if(area <= 100.0) continue;
        cv::Rect box = cv::boundingRect(shapes[i]);
        std::vector<cv::Point> hull;
        cv::convexHull(shapes[i], hull);
        if(box.height == 0 || hull.empty()) continue;
        double aspect_ratio = static_cast<double>(box.width) / box.height;
        double solidity = area / cv::contourArea(hull);
        if(aspect_ratio < 1.0f - shape_tolerance || aspect_ratio > 1.0f + shape_tolerance || solidity < 1.0f - shape_tolerance) continue;
        cv::Rect contour = cv::boundingRect(shapes[i]);
        cv::Point shape_center(contour.x + contour.width/2, contour.y + contour.height/2);
        //double dis = cv::norm(shape_center - sector_center); // prob not working as intended
        double dis = (shape_center.x-sector_center.x)*(shape_center.x-sector_center.x) + (shape_center.y-sector_center.y)*(shape_center.y-sector_center.y);
        if(dis < min_dis){
            min_dis = dis;
            shape = shapes[i];
        }
    }
    if(min_dis == DBL_MAX)
        return placeText("NO SHAPE", frame);

    cv::Rect box = cv::boundingRect(shape), final;
    final.x = box.x + sector.x - 10;
    final.y = box.y + sector.y - 10;
    final.width = box.width + 20;
    final.height = box.height + 20;
    cv::rectangle(frame, final, cv::Scalar(0, 255, 0), 5);
    return frame;
}