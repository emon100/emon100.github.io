const translations = {
    zh: {
        "title": "王一蒙 - 全栈开发者",
        "welcome": "欢迎来到我的个人主页",
        "intro": "我是王一蒙，一名热衷于创造创新解决方案的全栈开发者。",
        "nav.projects": "项目",
        "nav.skills": "技能",
        "nav.contact": "联系",
        "resume.title": "个人简历",
        "resume.education": "教育背景",
        "resume.project": "项目经验",
        "resume.experience": "工作经验",
        "resume.skills": "专业技能",
        "resume.technicalSkills": "技术技能",
        "resume.softSkills": "软技能",
        "projects.title": "精选项目",
        "projects.blog.title": "个人博客",
        "projects.blog.description": "基于WordPress的博客，展示我的想法和经验。",
        "projects.blog.button": "访问博客",
        "projects.github.title": "GitHub",
        "projects.github.description": "查看我的开源项目和贡献。",
        "projects.github.button": "访问GitHub",
        "projects.turingMachine.title": "通用图灵机",
        "projects.turingMachine.description": "探索图灵机的概念和实现。",
        "projects.turingMachine.button": "查看项目",
        "projects.gotoStudy.title": "GOTO语句研究",
        "projects.gotoStudy.description": "重新审视《GOTO 语句被认为有害》。",
        "projects.gotoStudy.button": "阅读文章",
        "projects.dataStructure1.title": "数据结构实验1",
        "projects.dataStructure1.description": "探索基础数据结构的实现。",
        "projects.dataStructure1.button": "查看实验",
        "projects.dataStructure5.title": "数据结构实验5",
        "projects.dataStructure5.description": "深入研究高级数据结构。",
        "projects.dataStructure5.button": "查看实验",
        "projects.webGL.title": "WebGL学习",
        "projects.webGL.description": "探索WebGL的基础知识和应用。",
        "projects.webGL.button": "查看项目",
        "contact.title": "联系我",
        "contact.description": "我始终对新的机会和合作持开放态度。欢迎随时联系我！",
        "footer.copyright": "&copy; 2023 王一蒙. 保留所有权利。",
        "friends.title": "友情链接",
        "friends.lian200.title": "Lian200's blog",
        "friends.lian200.description": "探索Lian200的思考和经验。",
        "friends.lian200.button": "访问博客"
    },
    en: {
        "title": "Yimeng Wang - Full Stack Developer",
        "welcome": "Welcome to My Personal Website",
        "intro": "I'm Yimeng Wang, a full-stack developer passionate about creating innovative solutions.",
        "nav.projects": "Projects",
        "nav.skills": "Skills",
        "nav.contact": "Contact",
        "resume.title": "Resume",
        "resume.education": "Education",
        "resume.project": "Project Experience",
        "resume.experience": "Work Experience",
        "resume.skills": "Professional Skills",
        "resume.technicalSkills": "Technical Skills",
        "resume.softSkills": "Soft Skills",
        "projects.title": "Featured Projects",
        "projects.blog.title": "Personal Blog",
        "projects.blog.description": "A WordPress-based blog showcasing my thoughts and experiences.",
        "projects.blog.button": "Visit Blog",
        "projects.github.title": "GitHub",
        "projects.github.description": "Check out my open-source projects and contributions.",
        "projects.github.button": "Visit GitHub",
        "projects.turingMachine.title": "Universal Turing Machine",
        "projects.turingMachine.description": "Explore the concept and implementation of Turing machines.",
        "projects.turingMachine.button": "View Project",
        "projects.gotoStudy.title": "GOTO Statement Study",
        "projects.gotoStudy.description": "Revisiting 'Go To Statement Considered Harmful'.",
        "projects.gotoStudy.button": "Read Article",
        "projects.dataStructure1.title": "Data Structure Experiment 1",
        "projects.dataStructure1.description": "Exploring the implementation of basic data structures.",
        "projects.dataStructure1.button": "View Experiment",
        "projects.dataStructure5.title": "Data Structure Experiment 5",
        "projects.dataStructure5.description": "In-depth study of advanced data structures.",
        "projects.dataStructure5.button": "View Experiment",
        "projects.webGL.title": "WebGL Learning",
        "projects.webGL.description": "Exploring the basics and applications of WebGL.",
        "projects.webGL.button": "View Project",
        "contact.title": "Contact Me",
        "contact.description": "I'm always open to new opportunities and collaborations. Feel free to reach out!",
        "footer.copyright": "&copy; 2023 Yimeng Wang. All rights reserved.",
        "friends.title": "Friend Links",
        "friends.lian200.title": "Lian200's blog",
        "friends.lian200.description": "Explore Lian200's thoughts and experiences.",
        "friends.lian200.button": "Visit Blog"
    }
};

function setLanguage(lang) {
    document.documentElement.lang = lang;
    document.querySelectorAll('[data-i18n]').forEach(element => {
        const key = element.getAttribute('data-i18n');
        if (translations[lang][key]) {
            element.innerHTML = translations[lang][key];
        }
    });
}

document.addEventListener('DOMContentLoaded', () => {
    const languageSelector = document.getElementById('languageSelector');
    languageSelector.addEventListener('change', (event) => {
        setLanguage(event.target.value);
    });

    // 初始化语言
    setLanguage('zh');
});