#pragma once
#include <limits>


struct Accelerator {
    bool null;
    guint accel_key;
    Gdk::ModifierType accel_mods;
    Glib::RefPtr<Gtk::AccelGroup> accel_group;

    Accelerator() : null(true) {}
    Accelerator(
        guint accel_key,
        Gdk::ModifierType accel_mods,
        const Glib::RefPtr<Gtk::AccelGroup>& accel_group
    )
        : null(false), accel_key(accel_key),
          accel_mods(accel_mods), accel_group(accel_group)
    {}
};


inline Gtk::MenuItem* createMenuItem(
    const std::string& itemName,
    std::function<void()> handler = nullptr,
    const Accelerator& accelerator = Accelerator()
) {
    Gtk::MenuItem* item = Gtk::manage(new Gtk::MenuItem(itemName, true));
    if (handler) {
        item->signal_activate().connect(handler);
    }

    if (!accelerator.null) {
        item->add_accelerator("activate", accelerator.accel_group,
                              accelerator.accel_key, accelerator.accel_mods,
                              Gtk::ACCEL_VISIBLE);
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
