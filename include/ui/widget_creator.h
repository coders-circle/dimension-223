#pragma once
#include <limits>


inline Gtk::MenuItem* createMenuItem(
    const std::string& itemName,
    std::function<void()> handler = nullptr
) {
    Gtk::MenuItem* item = Gtk::manage(new Gtk::MenuItem(itemName, true));
    if (handler) {
        item->signal_activate().connect(handler);
    }
    return item;
}


inline Gtk::SpinButton* createSpinEntry(
    double increment = 0.005, int digits = 3, int width = 6,
    double min = -std::numeric_limits<double>::max(),
    double max = std::numeric_limits<double>::max()
) {
    Gtk::SpinButton* entry = Gtk::manage(new Gtk::SpinButton());
    entry->set_range(min, max);
    entry->set_increments(increment, increment);
    entry->set_digits(digits);
    entry->set_width_chars(width);

    return entry;
}


inline Gtk::ToolItem* createToolItem(
    Gtk::Widget* widget
) {
    Gtk::ToolItem* item = Gtk::manage(new Gtk::ToolItem());
    item->add(*widget);
    return item;
}
