package org.firstinspires.ftc.teamcode.util.vars;


public class Section {
    public static final String VARIABLES_CATEGORY = "Variables";
    public static final String MODIFIERS_CATEGORY = "Modifiers";
    public String category;
    public String name;
    public boolean isModifier;
    public Section(String cat, String name, boolean isModifier) {
        this.category = cat;
        this.name = name;
        this.isModifier = isModifier;
    }
}
